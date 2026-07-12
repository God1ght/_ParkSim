"""Critical-state analysis for ParkSim-Qwen-VLA CSV/JSON outputs."""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List


def _read_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def _write_csv(path: Path, rows: Iterable[Dict[str, Any]], fields: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _num(row: Dict[str, str], key: str, default: float = 0.0) -> float:
    value = row.get(key)
    if value in (None, "", "nan", "None"):
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _pct(delta: float, baseline: float) -> str:
    if abs(baseline) < 1e-9:
        return ""
    return f"{delta / baseline * 100.0:.1f}%"


def _classify(qwen: Dict[str, str], baseline: Dict[str, str]) -> tuple[str, str]:
    objective_delta = _num(qwen, "objective_score") - _num(baseline, "objective_score")
    path_delta = _num(qwen, "fleet_total_automated_path_length") - _num(baseline, "fleet_total_automated_path_length")
    wait_delta = _num(qwen, "fleet_total_automated_waiting_time") - _num(baseline, "fleet_total_automated_waiting_time")
    near_delta = _num(qwen, "system_near_miss_event_count") - _num(baseline, "system_near_miss_event_count")
    collision_delta = _num(qwen, "system_collision_proxy_event_count") - _num(baseline, "system_collision_proxy_event_count")
    conflict_delta = _num(qwen, "trajectory_conflict_event_count") - _num(baseline, "trajectory_conflict_event_count")
    if objective_delta <= 0:
        return "path_coordination_gain", "Keep the successful bundle as a prompt/rule prior for similar scenes."
    if collision_delta > 3 or near_delta > 3:
        return "near_miss_or_collision_proxy_increase", "Increase bottleneck time-window risk weight and expose min distance, hidden intent, and yield cost to Qwen."
    if conflict_delta > 10:
        return "trajectory_conflict_accumulation", "Use fleet-level conflict matrices rather than independent shortest path candidates."
    if wait_delta > 5:
        return "waiting_recovery_cost_increase", "Add waiting age, blocking vehicle id, and replan deadline to the fleet state."
    if path_delta < 0:
        return "short_path_not_system_optimal", "Penalize short routes that traverse high-risk bottlenecks."
    return "mixed_objective_degradation", "Inspect per-window metrics and video frames to isolate the degradation window."


def _paired_drivers(rows: List[Dict[str, str]], baseline_agent: str, target_agent: str) -> List[Dict[str, Any]]:
    by_pair: Dict[str, Dict[str, Dict[str, str]]] = defaultdict(dict)
    for row in rows:
        key = row.get("pair_key") or row.get("scenario_id") or row.get("benchmark_id") or "unknown"
        by_pair[key][row.get("agent_type", "")] = row
    output = []
    for pair_key, agents in sorted(by_pair.items()):
        if baseline_agent not in agents or target_agent not in agents:
            continue
        base = agents[baseline_agent]
        qwen = agents[target_agent]
        reason, suggestion = _classify(qwen, base)
        obj_delta = _num(qwen, "objective_score") - _num(base, "objective_score")
        output.append({
            "pair_key": pair_key,
            "seed": qwen.get("seed", ""),
            "qwen_objective": f"{_num(qwen, 'objective_score'):.6f}",
            "baseline_objective": f"{_num(base, 'objective_score'):.6f}",
            "objective_delta_qwen_minus_baseline": f"{obj_delta:.6f}",
            "objective_delta_percent": _pct(obj_delta, _num(base, "objective_score")),
            "path_delta_m": f"{_num(qwen, 'fleet_total_automated_path_length') - _num(base, 'fleet_total_automated_path_length'):.6f}",
            "waiting_delta_s": f"{_num(qwen, 'fleet_total_automated_waiting_time') - _num(base, 'fleet_total_automated_waiting_time'):.6f}",
            "near_miss_delta": f"{_num(qwen, 'system_near_miss_event_count') - _num(base, 'system_near_miss_event_count'):.6f}",
            "collision_proxy_delta": f"{_num(qwen, 'system_collision_proxy_event_count') - _num(base, 'system_collision_proxy_event_count'):.6f}",
            "trajectory_conflict_delta": f"{_num(qwen, 'trajectory_conflict_event_count') - _num(base, 'trajectory_conflict_event_count'):.6f}",
            "traffic_scheduled_count": f"{_num(qwen, 'traffic_scheduled_count'):.0f}",
            "traffic_spawned_count": f"{_num(qwen, 'traffic_spawned_count'):.0f}",
            "traffic_delayed_count": f"{_num(qwen, 'traffic_delayed_count'):.0f}",
            "hidden_intent_event_count": f"{_num(qwen, 'traffic_hidden_event_count'):.0f}",
            "critical_driver": reason,
            "optimization_suggestion": suggestion,
        })
    return output


def _jsonl(path: Path):
    with path.open() as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                yield json.loads(line)
            except json.JSONDecodeError:
                continue


def _feature(decision: Dict[str, Any], key: str) -> float:
    features = ((decision.get("action_bundle") or {}).get("features") or {})
    try:
        return float(features.get(key) or 0.0)
    except (TypeError, ValueError):
        return 0.0


def _critical_events(video_dir: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if not video_dir.exists():
        return rows
    for path in sorted(video_dir.glob("**/fleet_epochs.jsonl")):
        case_name = "/".join(path.relative_to(video_dir).parts[:-1])
        for epoch in _jsonl(path):
            decisions = epoch.get("fleet_decisions") or []
            if not decisions:
                continue
            checks = []
            fallback = [d for d in decisions if d.get("used_fallback") or "duplicate" in str(d.get("reason", "")).lower()]
            risky = [d for d in decisions if _feature(d, "conflict_risk") >= 0.8 or _feature(d, "conflict_vehicle_count") >= 3]
            rejected = [d for d in decisions if d.get("shield_ok") is False]
            all_wait = decisions if len(decisions) >= 2 and all(str(d.get("action_id", "")).startswith("wait") for d in decisions) else []
            if fallback:
                checks.append(("fallback_or_duplicate_target", fallback, "high"))
            if risky:
                checks.append(("high_conflict_action_executed", risky, "high"))
            if rejected:
                checks.append(("shield_rejection", rejected, "high"))
            if all_wait:
                checks.append(("fleet_synchronized_wait", all_wait, "medium"))
            for event_type, selected, severity in checks:
                rows.append({
                    "case": case_name,
                    "sim_time": f"{float(epoch.get('sim_time') or 0.0):.3f}",
                    "fleet_epoch": epoch.get("epoch_id", ""),
                    "event_type": event_type,
                    "severity": severity,
                    "vehicle_ids": ",".join(str(d.get("vehicle_id", "")) for d in selected),
                    "action_ids": ",".join(str(d.get("action_id", "")) for d in selected),
                    "max_conflict_risk": f"{max(_feature(d, 'conflict_risk') for d in selected):.3f}",
                    "max_conflict_vehicle_count": f"{max(_feature(d, 'conflict_vehicle_count') for d in selected):.0f}",
                    "fallback_count": str(sum(1 for d in selected if d.get("used_fallback"))),
                    "video_lookup": f"{path.parent.parent / 'rule_vs_qwen_vla.mp4'} @ sim_time={float(epoch.get('sim_time') or 0.0):.1f}s",
                })
    return rows


def _write_summary(path: Path, drivers: List[Dict[str, Any]], events: List[Dict[str, Any]]) -> None:
    driver_counts = Counter(row["critical_driver"] for row in drivers)
    event_counts = Counter(row["event_type"] for row in events)
    lines = [
        "# Critical state analysis",
        "",
        "This report is generated from CSV/JSON logs. It does not modify manuscript text.",
        "",
        "## Metric drivers",
    ]
    for key, count in driver_counts.most_common():
        lines.append(f"- {key}: {count}")
    lines.extend(["", "## Critical state events"])
    for key, count in event_counts.most_common():
        lines.append(f"- {key}: {count}")
    if not event_counts:
        lines.append("- none detected by current rules")
    path.write_text("\n".join(lines) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate critical-state CSVs from ParkSim VLA reports.")
    parser.add_argument("--report-dir", required=True, type=Path)
    parser.add_argument("--video-dir", type=Path)
    parser.add_argument("--out-dir", required=True, type=Path)
    parser.add_argument("--baseline-agent", default="rule_based")
    parser.add_argument("--target-agent", default="qwen_vla")
    args = parser.parse_args()
    report_rows = _read_csv(args.report_dir / "paper_rows.csv")
    drivers = _paired_drivers(report_rows, args.baseline_agent, args.target_agent)
    events = _critical_events(args.video_dir) if args.video_dir else []
    args.out_dir.mkdir(parents=True, exist_ok=True)
    _write_csv(args.out_dir / "critical_metric_drivers.csv", drivers, [
        "pair_key", "seed", "qwen_objective", "baseline_objective",
        "objective_delta_qwen_minus_baseline", "objective_delta_percent", "path_delta_m",
        "waiting_delta_s", "near_miss_delta", "collision_proxy_delta",
        "trajectory_conflict_delta", "traffic_scheduled_count", "traffic_spawned_count",
        "traffic_delayed_count", "hidden_intent_event_count", "critical_driver",
        "optimization_suggestion",
    ])
    _write_csv(args.out_dir / "critical_state_events.csv", events, [
        "case", "sim_time", "fleet_epoch", "event_type", "severity", "vehicle_ids",
        "action_ids", "max_conflict_risk", "max_conflict_vehicle_count", "fallback_count",
        "video_lookup",
    ])
    _write_summary(args.out_dir / "critical_state_analysis.md", drivers, events)
    print(f"critical_state_analysis={args.out_dir} drivers={len(drivers)} events={len(events)}")


if __name__ == "__main__":
    main()
