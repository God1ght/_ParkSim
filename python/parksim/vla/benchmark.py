import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

from parksim.vla.compare_results import collect_mode_metrics
from parksim.vla.decision_audit import audit_logs, discover_logs, write_outputs

PREFERRED_FIELDS = [
    "scenario_id", "agent_type", "seed", "background_mode", "spot_index",
    "completed", "objective_score", "total_time", "total_non_idle_time",
    "path_length", "idle_time", "low_speed_time", "waiting_time", "decision_count",
    "qwen_fallback_count", "shield_rejection_count", "qwen_latency_mean",
    "selected_spot_index", "executed_spot_index", "first_action_type",
    "other_vehicle_trace_count", "min_other_distance_m", "min_ttc_s",
    "near_miss_event_count", "near_miss_time_s", "collision_proxy_event_count", "collision_proxy_time_s",
    "unsafe_occupancy_action_count", "malformed_decision_count", "target_mismatch_decision_count", "missing_reason_code_count",
    "candidate_action_count_mean", "available_candidate_count_mean", "blocked_candidate_count_mean",
    "trace_path", "summary_path", "decisions_path",
]


def _load_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    with path.open() as f:
        return json.load(f)


def _load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        json.dump(payload, f, indent=2)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _mean(values: Iterable[float]) -> float:
    values = list(values)
    return sum(values) / len(values) if values else 0.0


def _mean_or_none(values: Iterable[float]) -> Optional[float]:
    values = list(values)
    return sum(values) / len(values) if values else None


def _format_metric(value: Any) -> str:
    if value is None:
        return "NA"
    try:
        return "%.3f" % float(value)
    except Exception:
        return str(value)


def objective_score(metrics: Dict[str, Any]) -> float:
    incomplete_penalty = 0.0 if metrics.get("completed") else 1000.0
    return (
        incomplete_penalty
        + _safe_float(metrics.get("path_length"))
        + 0.2 * _safe_float(metrics.get("total_non_idle_time"))
        + 0.1 * _safe_float(metrics.get("idle_time"))
        + 500.0 * _safe_float(metrics.get("collision_proxy_event_count"))
        + 100.0 * _safe_float(metrics.get("near_miss_event_count"))
        + 100.0 * _safe_float(metrics.get("unsafe_occupancy_action_count"))
        + 20.0 * _safe_float(metrics.get("malformed_decision_count"))
        + 20.0 * _safe_float(metrics.get("target_mismatch_decision_count"))
        + 2.0 * _safe_float(metrics.get("missing_reason_code_count"))
        + 5.0 * _safe_float(metrics.get("shield_rejection_count"))
        + 2.0 * _safe_float(metrics.get("qwen_fallback_count"))
        + 0.05 * _safe_float(metrics.get("qwen_latency_mean"))
    )


def collect(out_dir: Path, root: Path) -> List[Dict[str, Any]]:
    del root
    episode_rows = _load_jsonl(out_dir / "episodes.jsonl")
    collected: List[Dict[str, Any]] = []
    for episode in episode_rows:
        scenario_dir = Path(episode.get("scenario_dir", ""))
        if not scenario_dir.is_absolute():
            scenario_dir = out_dir / "episodes" / str(episode["scenario_id"])
        agent_type = str(episode["agent_type"])
        item = collect_mode_metrics(scenario_dir, agent_type)
        metrics = item["metrics"]
        row = dict(episode)
        row.update(metrics)
        row["objective_score"] = objective_score(metrics)
        row["scenario_dir"] = str(scenario_dir)
        collected.append(row)
    return collected


def write_metrics(out_dir: Path, rows: List[Dict[str, Any]]) -> None:
    _write_json(out_dir / "metrics.json", rows)
    if not rows:
        (out_dir / "metrics.csv").write_text("")
        return
    extra = sorted(key for row in rows for key in row if key not in PREFERRED_FIELDS)
    fields = PREFERRED_FIELDS + extra
    with (out_dir / "metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def aggregate_by_agent(rows: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("agent_type"))].append(row)
    summary: List[Dict[str, Any]] = []
    for agent_type in sorted(grouped):
        group = grouped[agent_type]
        completed = [row for row in group if row.get("completed")]
        summary.append({
            "agent_type": agent_type,
            "episodes": len(group),
            "success_rate": len(completed) / len(group) if group else 0.0,
            "completed_count": len(completed),
            "mean_total_time": _mean(_safe_float(row.get("total_time")) for row in group),
            "mean_total_non_idle_time": _mean(_safe_float(row.get("total_non_idle_time")) for row in group),
            "mean_path_length": _mean(_safe_float(row.get("path_length")) for row in group),
            "mean_idle_time": _mean(_safe_float(row.get("idle_time")) for row in group),
            "mean_low_speed_time": _mean(_safe_float(row.get("low_speed_time")) for row in group),
            "mean_waiting_time": _mean(_safe_float(row.get("waiting_time")) for row in group),
            "mean_decision_count": _mean(_safe_float(row.get("decision_count")) for row in group),
            "mean_fallback_count": _mean(_safe_float(row.get("qwen_fallback_count")) for row in group),
            "mean_shield_rejection_count": _mean(_safe_float(row.get("shield_rejection_count")) for row in group),
            "mean_latency_s": _mean(_safe_float(row.get("qwen_latency_mean")) for row in group),
            "mean_min_other_distance_m": _mean_or_none(_safe_float(row.get("min_other_distance_m")) for row in group if row.get("min_other_distance_m") is not None),
            "mean_near_miss_events": _mean(_safe_float(row.get("near_miss_event_count")) for row in group),
            "mean_collision_proxy_events": _mean(_safe_float(row.get("collision_proxy_event_count")) for row in group),
            "mean_unsafe_occupancy_actions": _mean(_safe_float(row.get("unsafe_occupancy_action_count")) for row in group),
            "mean_target_mismatch_decisions": _mean(_safe_float(row.get("target_mismatch_decision_count")) for row in group),
            "mean_missing_reason_code": _mean(_safe_float(row.get("missing_reason_code_count")) for row in group),
            "mean_objective_score": _mean(_safe_float(row.get("objective_score")) for row in group),
        })
    return summary


def write_summary(out_dir: Path, rows: List[Dict[str, Any]]) -> None:
    grouped = aggregate_by_agent(rows)
    _write_json(out_dir / "summary.json", grouped)
    lines = [
        "# ParkSim-VLA-Bench Summary",
        "",
        "| agent | episodes | success | objective | path_m | non_idle_s | min_dist_m | near_miss | collision_proxy | unsafe_spot | decisions | latency_s |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in grouped:
        lines.append(
            "| %s | %d | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |"
            % (
                row.get("agent_type", ""),
                int(row.get("episodes", 0)),
                _format_metric(row.get("success_rate")),
                _format_metric(row.get("mean_objective_score")),
                _format_metric(row.get("mean_path_length")),
                _format_metric(row.get("mean_total_non_idle_time")),
                _format_metric(row.get("mean_min_other_distance_m")),
                _format_metric(row.get("mean_near_miss_events")),
                _format_metric(row.get("mean_collision_proxy_events")),
                _format_metric(row.get("mean_unsafe_occupancy_actions")),
                _format_metric(row.get("mean_decision_count")),
                _format_metric(row.get("mean_latency_s")),
            )
        )
    lines.extend([
        "",
        "Objective score is lower-is-better: path/time cost plus incomplete, near-miss, collision-proxy, unsafe-spot, malformed, shield, fallback, and latency penalties.",
        "Use `metrics.csv` for statistical tests and `preference_dataset.jsonl` for Qwen policy optimization data.",
        "Use `validation.json` and `manifest.json` to verify artifact completeness before including a run in paper tables.",
    ])
    (out_dir / "summary.md").write_text("\n".join(lines) + "\n")


def write_preference_dataset(out_dir: Path, rows: List[Dict[str, Any]]) -> None:
    by_scenario: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_scenario[str(row.get("scenario_id"))].append(row)
    path = out_dir / "preference_dataset.jsonl"
    with path.open("w") as f:
        for scenario_id, group in sorted(by_scenario.items()):
            valid = [row for row in group if _safe_float(row.get("trace_points")) > 0]
            if len(valid) < 2:
                continue
            chosen = min(valid, key=lambda row: _safe_float(row.get("objective_score"), 1e9))
            for rejected in valid:
                if rejected is chosen:
                    continue
                record = {
                    "scenario_id": scenario_id,
                    "chosen_agent": chosen.get("agent_type"),
                    "rejected_agent": rejected.get("agent_type"),
                    "objective": "minimize incomplete-penalized path/time/safety/shield/fallback score",
                    "chosen": _preference_view(chosen),
                    "rejected": _preference_view(rejected),
                }
                f.write(json.dumps(record) + "\n")


def write_decision_audit(out_dir: Path) -> Dict[str, Any]:
    logs = discover_logs([out_dir])
    summary = audit_logs(logs)
    write_outputs(summary, out_dir)
    return summary


def _preference_view(row: Dict[str, Any]) -> Dict[str, Any]:
    keys = [
        "completed", "objective_score", "path_length", "total_non_idle_time",
        "idle_time", "decision_count", "qwen_fallback_count", "shield_rejection_count",
        "min_other_distance_m", "near_miss_event_count", "collision_proxy_event_count",
        "unsafe_occupancy_action_count", "malformed_decision_count", "target_mismatch_decision_count", "missing_reason_code_count",
        "selected_spot_index", "executed_spot_index", "first_action_type",
        "decisions_path", "trace_path",
    ]
    return {key: row.get(key) for key in keys}


def validate(out_dir: Path, require_complete: bool = False, expected_agents: Optional[List[str]] = None) -> Dict[str, Any]:
    expected_agents = expected_agents or []
    problems: List[str] = []
    warnings: List[str] = []
    episodes = _load_jsonl(out_dir / "episodes.jsonl")
    metrics_rows = _load_json(out_dir / "metrics.json")
    if not episodes:
        problems.append("episodes.jsonl is missing or empty")
    if not isinstance(metrics_rows, list) or len(metrics_rows) != len(episodes):
        problems.append("metrics.json row count does not match episodes.jsonl")
    for artifact in ("metrics.csv", "metrics.json", "summary.md", "summary.json", "preference_dataset.jsonl"):
        if not (out_dir / artifact).exists():
            problems.append("missing artifact: %s" % artifact)
    audit_path = out_dir / "decision_audit.json"
    has_qwen_episode = any(str(episode.get("agent_type", "")).startswith("qwen") for episode in episodes)
    if audit_path.exists():
        audit = _load_json(audit_path)
        if not audit.get("ok", False):
            problems.append("decision audit failed: %s failures" % audit.get("failure_count", "unknown"))
    elif has_qwen_episode:
        warnings.append("missing decision_audit.json for qwen agent episodes")
    by_scenario: Dict[str, set] = defaultdict(set)
    for episode in episodes:
        scenario_id = str(episode.get("scenario_id", ""))
        agent = str(episode.get("agent_type", ""))
        by_scenario[scenario_id].add(agent)
        log_dir = Path(str(episode.get("log_dir", "")))
        if not log_dir.exists():
            problems.append("missing log_dir for %s/%s: %s" % (scenario_id, agent, log_dir))
            continue
        if not list(log_dir.glob("vehicle_*_trace.jsonl")):
            problems.append("missing vehicle trace for %s/%s" % (scenario_id, agent))
        if not list(log_dir.glob("vehicle_*_summary.json")):
            warnings.append("missing vehicle summary for %s/%s" % (scenario_id, agent))
    for row in metrics_rows if isinstance(metrics_rows, list) else []:
        if _safe_float(row.get("trace_points")) <= 0:
            problems.append("metric row has no trace points: %s/%s" % (row.get("scenario_id"), row.get("agent_type")))
        if require_complete and not row.get("completed"):
            problems.append("incomplete episode under --require-complete: %s/%s" % (row.get("scenario_id"), row.get("agent_type")))
    if expected_agents:
        expected = set(expected_agents)
        for scenario_id, agents in by_scenario.items():
            missing = sorted(expected - agents)
            if missing:
                problems.append("scenario %s missing expected agents: %s" % (scenario_id, ",".join(missing)))
    result = {
        "ok": not problems,
        "problems": problems,
        "warnings": warnings,
        "episode_count": len(episodes),
        "metric_count": len(metrics_rows) if isinstance(metrics_rows, list) else 0,
        "scenario_count": len(by_scenario),
        "expected_agents": expected_agents,
        "require_complete": bool(require_complete),
    }
    _write_json(out_dir / "validation.json", result)
    return result


def write_manifest(out_dir: Path, rows: List[Dict[str, Any]], validation: Dict[str, Any]) -> None:
    artifacts = {}
    for name in ("episodes.jsonl", "metrics.csv", "metrics.json", "summary.md", "summary.json", "preference_dataset.jsonl", "decision_audit.json", "decision_audit_failures.jsonl", "validation.json"):
        path = out_dir / name
        artifacts[name] = {"path": str(path), "exists": path.exists(), "bytes": path.stat().st_size if path.exists() else 0}
    manifest = {
        "type": "parksim_vla_benchmark",
        "out_dir": str(out_dir),
        "run_config": _load_json(out_dir / "run_config.json"),
        "artifacts": artifacts,
        "episode_count": len(rows),
        "agents": sorted({str(row.get("agent_type")) for row in rows}),
        "scenarios": sorted({str(row.get("scenario_id")) for row in rows}),
        "validation": validation,
    }
    _write_json(out_dir / "manifest.json", manifest)


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect and validate reproducible ParkSim VLA benchmark metrics.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    collect_parser = sub.add_parser("collect")
    collect_parser.add_argument("out_dir", type=Path)
    collect_parser.add_argument("--root", type=Path, default=Path.cwd())
    validate_parser = sub.add_parser("validate")
    validate_parser.add_argument("out_dir", type=Path)
    validate_parser.add_argument("--require-complete", action="store_true")
    validate_parser.add_argument("--expected-agents", default="")
    args = parser.parse_args()

    if args.cmd == "collect":
        out_dir = args.out_dir.resolve()
        root = args.root.resolve()
        rows = collect(out_dir, root)
        write_metrics(out_dir, rows)
        write_summary(out_dir, rows)
        write_preference_dataset(out_dir, rows)
        audit = write_decision_audit(out_dir)
        validation = validate(out_dir)
        write_manifest(out_dir, rows, validation)
        print("benchmark metrics written to %s" % out_dir)
        for row in aggregate_by_agent(rows):
            print("{agent_type} episodes={episodes} success={success_rate:.3f} objective={mean_objective_score:.3f}".format(**row))
        print("validation_ok=%s" % validation["ok"])
        print("decision_audit_ok=%s failures=%s logs=%s" % (audit.get("ok"), audit.get("failure_count"), audit.get("log_count")))
    elif args.cmd == "validate":
        out_dir = args.out_dir.resolve()
        expected = [item for item in args.expected_agents.split(",") if item]
        result = validate(out_dir, require_complete=args.require_complete, expected_agents=expected)
        print(json.dumps(result, indent=2))
        raise SystemExit(0 if result["ok"] else 1)


if __name__ == "__main__":
    main()
