import argparse
import csv
import hashlib
import json
import math
import pickle
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from parksim.vla.safety_metrics import collect_safety_metrics

MODES = ("rule_based", "qwen_vla")


def _load_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with path.open() as f:
        return json.load(f)


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _sum_interval(rows: List[Dict[str, Any]], predicate) -> float:
    total = 0.0
    for prev, cur in zip(rows, rows[1:]):
        dt = max(0.0, float(cur.get("time", 0.0)) - float(prev.get("time", 0.0)))
        if predicate(prev):
            total += dt
    return total


def _path_length(rows: List[Dict[str, Any]]) -> float:
    total = 0.0
    for prev, cur in zip(rows, rows[1:]):
        dx = float(cur.get("x", 0.0)) - float(prev.get("x", 0.0))
        dy = float(cur.get("y", 0.0)) - float(prev.get("y", 0.0))
        total += math.hypot(dx, dy)
    return total


def _mean(values: Iterable[float]) -> float:
    values = list(values)
    return sum(values) / len(values) if values else 0.0


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _stable_hash(payload: Any) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _traffic_schedule_hash(log_dir: Path) -> str:
    payload = _load_json(log_dir / "traffic_schedule.json")
    events = payload.get("events", []) if isinstance(payload, dict) else []
    canonical = []
    for event in events if isinstance(events, list) else []:
        if not isinstance(event, dict):
            continue
        canonical.append({
            "event_id": event.get("event_id"),
            "time": event.get("time"),
            "event_type": event.get("event_type"),
            "source": event.get("source"),
            "actor_class": event.get("actor_class", "human"),
            "spot_index": event.get("spot_index"),
            "intent_observable": event.get("intent_observable"),
            "preference_seed": event.get("preference_seed"),
        })
    return _stable_hash(canonical)


def _load_spots(root: Path) -> List[List[float]]:
    path = root / "python" / "parksim" / "priorFiles" / "spots_data.pickle"
    if not path.exists():
        return []
    try:
        with path.open("rb") as f:
            data = pickle.load(f)
        spots = data.get("parking_spaces", [])
        return [[float(row[0]), float(row[1])] for row in spots]
    except Exception:
        return []


def _latest_vehicle_file(log_dir: Path, suffix: str) -> Optional[Path]:
    files = sorted(log_dir.glob("vehicle_*_" + suffix))
    return files[0] if files else None


def _trace_for_summary(summary_path: Path) -> Path:
    return summary_path.with_name(summary_path.name.replace("_summary.json", "_trace.jsonl"))


def _score_ego_candidate(payload: Dict[str, Any], mode: str) -> int:
    if payload.get("is_controlled_ego") is True:
        return 100
    score = 0
    if str(payload.get("agent_type", "")).lower() == str(mode).lower():
        score += 10
    try:
        if int(payload.get("spot_index", 0) or 0) > 0:
            score += 3
    except Exception:
        pass
    if payload.get("completed") is True or payload.get("is_final") is True:
        score += 1
    return score


def _select_ego_files(log_dir: Path, mode: str) -> Tuple[Path, Path, Dict[str, Any], List[Dict[str, Any]]]:
    summary_candidates = []
    for summary_path in sorted(log_dir.glob("vehicle_*_summary.json")):
        payload = _load_json(summary_path)
        if not payload:
            continue
        score = _score_ego_candidate(payload, mode)
        summary_candidates.append((score, summary_path, payload))
    controlled_trace_candidates = []
    for trace_path in sorted(log_dir.glob("vehicle_*_trace.jsonl")):
        rows = _load_jsonl(trace_path)
        if not rows:
            continue
        first = rows[0]
        final = rows[-1]
        if first.get("is_controlled_ego") is True or final.get("is_controlled_ego") is True:
            score = max(_score_ego_candidate(first, mode), _score_ego_candidate(final, mode))
            controlled_trace_candidates.append((score, trace_path, rows))
    if summary_candidates:
        summary_candidates.sort(key=lambda item: (-item[0], str(item[1])))
        if summary_candidates[0][0] >= 100:
            _, summary_path, summary = summary_candidates[0]
            trace_path = _trace_for_summary(summary_path)
            return trace_path, summary_path, summary, _load_jsonl(trace_path)
    if controlled_trace_candidates:
        controlled_trace_candidates.sort(key=lambda item: (-item[0], str(item[1])))
        _, trace_path, trace = controlled_trace_candidates[0]
        summary_path = trace_path.with_name(trace_path.name.replace("_trace.jsonl", "_summary.json"))
        return trace_path, summary_path, _load_json(summary_path), trace
    if summary_candidates:
        _, summary_path, summary = summary_candidates[0]
        trace_path = _trace_for_summary(summary_path)
        return trace_path, summary_path, summary, _load_jsonl(trace_path)

    trace_candidates = []
    for trace_path in sorted(log_dir.glob("vehicle_*_trace.jsonl")):
        rows = _load_jsonl(trace_path)
        if not rows:
            continue
        final = rows[-1]
        first = rows[0]
        score = max(_score_ego_candidate(first, mode), _score_ego_candidate(final, mode))
        trace_candidates.append((score, trace_path, rows))
    if trace_candidates:
        trace_candidates.sort(key=lambda item: (-item[0], str(item[1])))
        _, trace_path, trace = trace_candidates[0]
        summary_path = trace_path.with_name(trace_path.name.replace("_trace.jsonl", "_summary.json"))
        return trace_path, summary_path, _load_json(summary_path), trace

    trace_path = log_dir / "vehicle_1_trace.jsonl"
    summary_path = log_dir / "vehicle_1_summary.json"
    return trace_path, summary_path, _load_json(summary_path), _load_jsonl(trace_path)


def collect_mode_metrics(experiment_dir: Path, mode: str) -> Dict[str, Any]:
    mode_dir = experiment_dir / mode
    log_dir = mode_dir / "logs"
    trace_path, summary_path, summary, trace = _select_ego_files(log_dir, mode)
    decisions_path = log_dir / "qwen_vla_decisions.jsonl"

    decisions = _load_jsonl(decisions_path)
    fleet_epochs_path = mode_dir / "fleet_epochs.jsonl"
    fleet_epochs = _load_jsonl(fleet_epochs_path)

    speeds = [_safe_float(row.get("speed")) for row in trace]
    steering = [_safe_float(row.get("steering")) for row in trace]
    accelerations = [_safe_float(row.get("acceleration")) for row in trace]
    final = trace[-1] if trace else {}
    first = trace[0] if trace else {}
    first_fleet_epoch = fleet_epochs[0] if fleet_epochs else {}
    first_fleet_context = first_fleet_epoch.get("fleet_context") or {}

    qwen_latencies = [_safe_float(row.get("latency_seconds")) for row in decisions]
    fleet_latencies = [_safe_float(row.get("latency_seconds")) for row in fleet_epochs]
    fleet_batch_waits = [_safe_float(row.get("batch_wait_seconds")) for row in fleet_epochs]
    barrier_epochs = [row for row in fleet_epochs if row.get("decision_barrier_status")]
    barrier_expected = sum(len(row.get("decision_ack_expected_vehicle_ids") or []) for row in barrier_epochs)
    barrier_received = sum(len(row.get("decision_ack_received_vehicle_ids") or []) for row in barrier_epochs)
    barrier_failures = [
        row for row in barrier_epochs
        if row.get("decision_barrier_status") not in ("applied", "no_action_required")
    ]
    barrier_time_mismatches = sum(
        1
        for row in barrier_epochs
        for ack in row.get("decision_acknowledgements") or []
        if "frozen simulation-time barrier" in str(ack.get("error", ""))
    )
    pre_critiques = [row.get("pre_feedback_critique") or {} for row in fleet_epochs]
    post_critiques = [row.get("post_feedback_critique") or {} for row in fleet_epochs]
    belief_rows = [
        belief
        for epoch in fleet_epochs
        for belief in (((epoch.get("fleet_context") or {}).get("state") or {}).get("human_intent_beliefs") or [])
        if isinstance(belief, dict)
    ]
    observability_audits = [
        item.get("observability_audit")
        for item in pre_critiques
        if isinstance(item.get("observability_audit"), dict)
        and "observability_audit_ok" in item.get("observability_audit", {})
    ]
    if fleet_latencies:
        qwen_latencies = fleet_latencies
    applied_actions = [row.get("applied_action") or {} for row in decisions]
    selected_spots = [action.get("target_spot_index") for action in applied_actions if action.get("target_spot_index") is not None]
    action_types = [action.get("action_type") for action in applied_actions if action.get("action_type")]

    metrics = {
        "mode": mode,
        "ego_vehicle_id": int(summary.get("vehicle_id", final.get("vehicle_id", 0)) or 0),
        "ego_is_controlled": bool(summary.get("is_controlled_ego", final.get("is_controlled_ego", False))),
        "metric_scope": "controlled_ego" if bool(summary.get("is_controlled_ego", final.get("is_controlled_ego", False))) else "fleet_member",
        "representative_vehicle_role": str(summary.get("vehicle_role", final.get("vehicle_role", first.get("vehicle_role", ""))) or ""),
        "representative_agent_type": str(summary.get("agent_type", final.get("agent_type", first.get("agent_type", ""))) or ""),
        "completed": bool(summary.get("completed", bool(final.get("is_final")))) if (summary or final) else False,
        "assigned_spot_index": int(summary.get("spot_index", final.get("spot_index", 0)) or 0),
        "executed_spot_index": int(summary.get("vehicle_spot_index", final.get("vehicle_spot_index", 0)) or 0),
        "selected_spot_index": int(selected_spots[0]) if selected_spots else None,
        "trace_points": len(trace),
        "total_time": _safe_float(summary.get("total_time"), _safe_float(final.get("time"))),
        "total_non_idle_time": _safe_float(summary.get("total_non_idle_time"), _safe_float(final.get("total_non_idle_time"))),
        "path_length": _path_length(trace),
        "mean_speed": _mean(abs(v) for v in speeds),
        "max_speed": max((abs(v) for v in speeds), default=0.0),
        "mean_abs_steering": _mean(abs(v) for v in steering),
        "mean_abs_acceleration": _mean(abs(v) for v in accelerations),
        "idle_time": _sum_interval(trace, lambda row: row.get("task") == "IDLE"),
        "low_speed_time": _sum_interval(trace, lambda row: abs(_safe_float(row.get("speed"))) < 0.1),
        "brake_time": _sum_interval(trace, lambda row: bool(row.get("is_braking"))),
        "waiting_time": _sum_interval(trace, lambda row: int(row.get("waiting_for", 0) or 0) > 0),
        "decision_count": len(decisions),
        "qwen_fallback_count": sum(1 for row in decisions if (row.get("decision") or {}).get("used_fallback")),
        "shield_rejection_count": sum(1 for row in decisions if row.get("shield_reason") != "ok"),
        "audit_qwen_wall_latency_mean": _mean(qwen_latencies),
        "audit_qwen_wall_latency_max": max(qwen_latencies, default=0.0),
        "audit_fleet_batch_wall_wait_mean": _mean(fleet_batch_waits),
        "audit_fleet_batch_wall_wait_max": max(fleet_batch_waits, default=0.0),
        "simulation_time_policy": str(first_fleet_epoch.get("simulation_time_policy", "")),
        "wall_clock_latency_in_performance_metrics": bool(first_fleet_epoch.get("wall_clock_latency_in_performance_metrics", False)),
        "decision_barrier_epoch_count": len(barrier_epochs),
        "decision_barrier_complete_count": len(barrier_epochs) - len(barrier_failures),
        "decision_barrier_failure_count": len(barrier_failures),
        "decision_barrier_ack_expected_count": barrier_expected,
        "decision_barrier_ack_received_count": barrier_received,
        "decision_barrier_ack_coverage_rate": barrier_received / barrier_expected if barrier_expected else (1.0 if barrier_epochs else 0.0),
        "decision_barrier_sim_time_mismatch_count": barrier_time_mismatches,
        "cloud_fleet_event_trigger_epoch_count": sum(1 for row in fleet_epochs if row.get("trigger_type") == "event"),
        "cloud_fleet_watchdog_epoch_count": sum(1 for row in fleet_epochs if row.get("trigger_type") == "watchdog"),
        "cloud_fleet_actionable_epoch_count": sum(1 for row in fleet_epochs if row.get("fleet_decisions")),
        "cloud_fleet_empty_epoch_count": sum(1 for row in fleet_epochs if not row.get("fleet_decisions")),
        "cloud_fleet_deferred_vehicle_count": sum(len(row.get("deferred_vehicle_ids") or []) for row in fleet_epochs),
        "policy_variant": str(fleet_epochs[0].get("policy_mode", mode) if fleet_epochs else mode),
        "model_id": str(first_fleet_epoch.get("model_id", "")),
        "model_revision": str(first_fleet_epoch.get("model_revision", "")),
        "fleet_protocol_version": str(first_fleet_context.get("protocol_version", "")),
        "prompt_version": str(first_fleet_context.get("prompt_version", "")),
        "traffic_schedule_hash": _traffic_schedule_hash(log_dir),
        "feedback_enabled": any(bool(row.get("repair_attempted")) for row in fleet_epochs),
        "feedback_repair_attempt_count": sum(1 for row in fleet_epochs if row.get("repair_attempted")),
        "feedback_repair_success_count": sum(1 for row in fleet_epochs if row.get("repair_success")),
        "feedback_changed_action_count": sum(int(row.get("changed_by_feedback_count", 0) or 0) for row in fleet_epochs),
        "audit_feedback_wall_latency_mean": _mean(_safe_float(row.get("repair_latency_seconds")) for row in fleet_epochs if row.get("repair_attempted")),
        "pre_feedback_hard_violation_count": sum(int(row.get("hard_violation_count", 0) or 0) for row in pre_critiques),
        "post_feedback_hard_violation_count": sum(int(row.get("hard_violation_count", 0) or 0) for row in post_critiques),
        "pre_feedback_quality_warning_count": sum(int(row.get("quality_warning_count", 0) or 0) for row in pre_critiques),
        "post_feedback_quality_warning_count": sum(int(row.get("quality_warning_count", 0) or 0) for row in post_critiques),
        "pre_feedback_route_conflict_count": sum(int(row.get("route_conflict_count", 0) or 0) for row in pre_critiques),
        "post_feedback_route_conflict_count": sum(int(row.get("route_conflict_count", 0) or 0) for row in post_critiques),
        "candidate_conflict_edge_count_mean": _mean(_safe_float(row.get("candidate_conflict_edge_count")) for row in pre_critiques),
        "human_belief_observation_count": len(belief_rows),
        "human_belief_entropy_mean": _mean(_safe_float(row.get("normalized_entropy")) for row in belief_rows),
        "observability_audit_ok": all(bool(row.get("observability_audit_ok")) for row in observability_audits) if observability_audits else None,
        "human_target_spot_exposed": any(bool(row.get("human_target_spot_exposed")) for row in observability_audits),
        "human_route_exposed": any(bool(row.get("human_route_exposed")) for row in observability_audits),
        "human_future_trajectory_exposed": any(bool(row.get("human_future_trajectory_exposed")) for row in observability_audits),
        "first_action_type": action_types[0] if action_types else None,
        "final_x": _safe_float((summary.get("final_state") or {}).get("x"), _safe_float(final.get("x"))),
        "final_y": _safe_float((summary.get("final_state") or {}).get("y"), _safe_float(final.get("y"))),
        "trace_path": str(trace_path),
        "summary_path": str(summary_path),
        "decisions_path": str(decisions_path) if decisions_path.exists() else "",
        "fleet_epochs_path": str(fleet_epochs_path) if fleet_epochs_path.exists() else "",
    }
    feedback_attempts = int(metrics.get("feedback_repair_attempt_count", 0) or 0)
    feedback_successes = int(metrics.get("feedback_repair_success_count", 0) or 0)
    metrics["feedback_repair_success_rate"] = feedback_successes / feedback_attempts if feedback_attempts else 0.0
    for issue in ("hard_violation", "quality_warning", "route_conflict"):
        pre = int(metrics.get("pre_feedback_%s_count" % issue, 0) or 0)
        post = int(metrics.get("post_feedback_%s_count" % issue, 0) or 0)
        reduction = pre - post
        metrics["feedback_%s_reduction_count" % issue] = reduction
        metrics["feedback_%s_reduction_rate" % issue] = reduction / pre if pre else 0.0
    metrics.update(collect_safety_metrics(log_dir, trace_path, trace, decisions))
    if fleet_epochs:
        requested = sum(len(row.get("collected_vehicle_ids") or []) for row in fleet_epochs)
        returned = sum(len(row.get("fleet_decisions") or []) for row in fleet_epochs)
        metrics.update({
            "cloud_fleet_decision_count": len(fleet_epochs),
            "cloud_fleet_vehicle_decision_count": returned,
            "cloud_fleet_missing_vehicle_decision_count": sum(len(row.get("missing_vehicle_ids") or []) for row in fleet_epochs),
            "cloud_fleet_requested_vehicle_decision_count": requested,
            "cloud_fleet_decision_coverage_rate": returned / requested if requested else 0.0,
        })
    return {"metrics": metrics, "trace": trace, "summary": summary, "decisions": decisions}


def write_metrics(experiment_dir: Path, collected: Dict[str, Dict[str, Any]]) -> None:
    rows = [collected[mode]["metrics"] for mode in MODES if mode in collected]
    with (experiment_dir / "metrics.json").open("w") as f:
        json.dump(rows, f, indent=2)
    fields = [
        "mode", "ego_vehicle_id", "ego_is_controlled", "completed", "assigned_spot_index", "executed_spot_index", "selected_spot_index",
        "total_time", "total_non_idle_time", "path_length", "mean_speed", "max_speed",
        "idle_time", "low_speed_time", "brake_time", "waiting_time", "decision_count",
        "qwen_fallback_count", "shield_rejection_count", "first_action_type",
        "other_vehicle_trace_count", "min_other_distance_m", "near_miss_event_count", "near_miss_time_s",
        "collision_proxy_event_count", "collision_proxy_time_s", "min_ttc_s", "unsafe_occupancy_action_count",
        "malformed_decision_count", "target_mismatch_decision_count", "missing_reason_code_count",
        "candidate_action_count_mean", "available_candidate_count_mean",
        "total_vehicle_trace_count", "completed_vehicle_count", "entering_vehicle_count", "exiting_vehicle_count",
        "automated_vehicle_count", "cloud_served_vehicle_count", "human_like_vehicle_count", "replay_vehicle_count",
        "human_rule_vehicle_count", "hidden_intent_vehicle_count", "cloud_fleet_decision_count",
        "cloud_fleet_vehicle_decision_count", "cloud_fleet_missing_vehicle_decision_count",
        "simulation_time_policy", "wall_clock_latency_in_performance_metrics", "decision_barrier_epoch_count",
        "decision_barrier_complete_count", "decision_barrier_failure_count", "decision_barrier_ack_expected_count",
        "decision_barrier_ack_received_count", "decision_barrier_ack_coverage_rate", "decision_barrier_sim_time_mismatch_count",
        "traffic_scheduled_count", "traffic_hidden_event_count",
        "traffic_spawned_count", "traffic_delayed_count", "traffic_skipped_count", "system_min_distance_m",
        "system_near_miss_event_count", "system_collision_proxy_event_count", "trajectory_conflict_event_count",
        "mixed_intent_conflict_event_count", "mixed_intent_conflict_time_s",
        "automated_demand_released_count", "automated_demand_service_rate", "automated_throughput_per_sim_hour",
        "system_exposure_vehicle_km", "system_exposure_vehicle_hours",
        "system_near_miss_events_per_100_vehicle_km", "system_near_miss_events_per_100_vehicle_hours",
        "system_collision_proxy_events_per_100_vehicle_km", "system_collision_proxy_events_per_100_vehicle_hours",
        "trajectory_conflicts_per_100_vehicle_km", "trajectory_conflicts_per_100_vehicle_hours",
        "feedback_repair_success_rate", "feedback_hard_violation_reduction_rate", "feedback_route_conflict_reduction_rate",
    ]
    with (experiment_dir / "metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def plot_trajectories(root: Path, experiment_dir: Path, collected: Dict[str, Dict[str, Any]]) -> None:
    spots = _load_spots(root)
    fig, ax = plt.subplots(figsize=(9, 9))
    if spots:
        ax.scatter([p[0] for p in spots], [p[1] for p in spots], s=8, c="#c9c9c9", label="parking spots", zorder=1)
    colors = {"rule_based": "#1f77b4", "qwen_vla": "#d62728"}
    for mode in MODES:
        item = collected.get(mode)
        if not item:
            continue
        trace = item["trace"]
        if not trace:
            continue
        xs = [_safe_float(row.get("x")) for row in trace]
        ys = [_safe_float(row.get("y")) for row in trace]
        metrics = item["metrics"]
        ax.plot(xs, ys, color=colors.get(mode), linewidth=2.0, label="%s path %.1fm" % (mode, metrics["path_length"]), zorder=3)
        ax.scatter(xs[0], ys[0], marker="o", s=55, color=colors.get(mode), edgecolor="black", zorder=4)
        ax.scatter(xs[-1], ys[-1], marker="x", s=80, color=colors.get(mode), zorder=4)
        target = metrics.get("selected_spot_index") or metrics.get("executed_spot_index") or metrics.get("assigned_spot_index")
        if spots and target is not None and 0 <= int(target) < len(spots):
            tx, ty = spots[int(target)]
            ax.scatter([tx], [ty], marker="*", s=160, color=colors.get(mode), edgecolor="black", zorder=5)
            ax.text(tx, ty, " %s:%s" % (mode, target), fontsize=8, color=colors.get(mode))
    ax.set_title("Rule-Based vs Qwen-VLA Trajectories")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.25)
    ax.axis("equal")
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(experiment_dir / "trajectories.png", dpi=160)
    plt.close(fig)


def plot_metrics(experiment_dir: Path, collected: Dict[str, Dict[str, Any]]) -> None:
    metric_names = [
        ("total_non_idle_time", "Non-idle time (s)"),
        ("path_length", "Path length (m)"),
        ("low_speed_time", "Low-speed time (s)"),
        ("near_miss_event_count", "Near-miss events"),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(10, 7))
    for ax, (name, title) in zip(axes.flat, metric_names):
        values = [collected.get(mode, {}).get("metrics", {}).get(name, 0.0) for mode in MODES]
        ax.bar(MODES, values, color=["#1f77b4", "#d62728"])
        ax.set_title(title)
        ax.grid(True, axis="y", alpha=0.25)
        for idx, value in enumerate(values):
            ax.text(idx, value, "%.2f" % value, ha="center", va="bottom", fontsize=8)
    fig.tight_layout()
    fig.savefig(experiment_dir / "metrics.png", dpi=160)
    plt.close(fig)


def write_summary(experiment_dir: Path, collected: Dict[str, Dict[str, Any]]) -> None:
    fields = [
        ("completed", "completed"),
        ("assigned_spot_index", "assigned_spot"),
        ("executed_spot_index", "executed_spot"),
        ("selected_spot_index", "selected_spot"),
        ("total_non_idle_time", "non_idle_s"),
        ("path_length", "path_m"),
        ("low_speed_time", "low_speed_s"),
        ("brake_time", "brake_s"),
        ("decision_count", "decisions"),
        ("qwen_fallback_count", "fallbacks"),
        ("shield_rejection_count", "shield_rejects"),
        ("decision_barrier_ack_coverage_rate", "decision_barrier_ack_coverage"),
        ("decision_barrier_failure_count", "decision_barrier_failures"),
        ("min_other_distance_m", "min_dist_m"),
        ("near_miss_event_count", "near_miss_events"),
        ("collision_proxy_event_count", "collision_proxy_events"),
        ("unsafe_occupancy_action_count", "unsafe_occupancy_actions"),
    ]
    lines = ["# ParkSim Policy Comparison", "", "| metric | rule_based | qwen_vla |", "| --- | ---: | ---: |"]
    for key, label in fields:
        vals = []
        for mode in MODES:
            value = collected.get(mode, {}).get("metrics", {}).get(key)
            if isinstance(value, float):
                vals.append("%.3f" % value)
            else:
                vals.append(str(value))
        lines.append("| %s | %s | %s |" % (label, vals[0], vals[1]))
    lines.extend([
        "",
        "Artifacts:",
        "- `trajectories.png`: ego path overlay with selected/target spots.",
        "- `metrics.png`: key metric comparison bars.",
        "- `metrics.csv` and `metrics.json`: machine-readable metrics.",
    ])
    (experiment_dir / "summary.md").write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Summarize and visualize rule_based vs qwen_vla ParkSim runs.")
    parser.add_argument("experiment_dir", type=Path)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    args = parser.parse_args()
    experiment_dir = args.experiment_dir.resolve()
    root = args.root.resolve()
    collected = {mode: collect_mode_metrics(experiment_dir, mode) for mode in MODES}
    write_metrics(experiment_dir, collected)
    plot_trajectories(root, experiment_dir, collected)
    plot_metrics(experiment_dir, collected)
    write_summary(experiment_dir, collected)
    print("comparison metrics written to %s" % experiment_dir)
    for mode in MODES:
        metrics = collected[mode]["metrics"]
        print("%s completed=%s path=%.2f non_idle=%.2f decisions=%d" % (
            mode,
            metrics["completed"],
            metrics["path_length"],
            metrics["total_non_idle_time"],
            metrics["decision_count"],
        ))


if __name__ == "__main__":
    main()
