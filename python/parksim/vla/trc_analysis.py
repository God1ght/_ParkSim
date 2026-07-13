import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


LOWER_IS_BETTER = {
    "objective_score",
    "fleet_total_automated_path_length",
    "fleet_total_automated_waiting_time",
    "fleet_total_automated_non_idle_time",
    "path_length",
    "total_non_idle_time",
    "waiting_time",
    "system_near_miss_event_count",
    "system_collision_proxy_event_count",
    "fleet_mean_automated_total_time",
    "system_near_miss_events_per_100_vehicle_km",
    "system_collision_proxy_events_per_100_vehicle_km",
    "trajectory_conflicts_per_100_vehicle_km",
    "mixed_intent_conflicts_per_100_vehicle_km",
    "trajectory_conflict_event_count",
    "mixed_intent_conflict_event_count",
    "unsafe_occupancy_action_count",
    "shield_rejection_count",
    "qwen_fallback_count",
    "decision_barrier_failure_count",
    "decision_barrier_sim_time_mismatch_count",
    "trace_integrity_invalid_file_count",
    "trace_identity_conflict_count",
    "trace_time_regression_count",
    "trace_wall_time_regression_count",
    "trace_kinematic_jump_count",
}

METRIC_GROUPS = {
    "fleet_efficiency": ["automated_demand_service_rate", "automated_throughput_per_sim_hour", "fleet_mean_automated_total_time", "fleet_total_automated_waiting_time", "objective_score"],
    "operational_safety": ["system_near_miss_events_per_100_vehicle_km", "system_collision_proxy_events_per_100_vehicle_km", "trajectory_conflicts_per_100_vehicle_km", "mixed_intent_conflicts_per_100_vehicle_km", "unsafe_occupancy_action_count"],
    "cloud_coordination": ["feedback_hard_violation_reduction_rate", "feedback_route_conflict_reduction_rate", "shield_rejection_count", "qwen_fallback_count", "cloud_fleet_missing_vehicle_decision_count"],
    "mixed_traffic_scale": ["automated_vehicle_count", "cloud_served_vehicle_count", "human_like_vehicle_count", "hidden_intent_vehicle_count"],
    "synchronous_decision_integrity": ["decision_barrier_ack_coverage_rate", "decision_barrier_failure_count", "decision_barrier_sim_time_mismatch_count"],
    "data_integrity": ["trace_integrity_ok", "trace_integrity_invalid_file_count", "trace_identity_conflict_count", "trace_time_regression_count", "trace_wall_time_regression_count", "trace_kinematic_jump_count"],
}


def _load_csv(path: Path) -> List[Dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def _load_json(path: Path, default: Any = None) -> Any:
    if not path.exists():
        return default
    with path.open() as f:
        return json.load(f)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value in (None, ""):
            return default
        return float(value)
    except Exception:
        return default


def _write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        path.write_text("")
        return
    fields = sorted({key for row in rows for key in row})
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})


def _metric_mean(row: Dict[str, str], metric: str) -> float:
    return _safe_float(row.get(metric + "_mean"))


def build_metric_group_summary(summary_rows: List[Dict[str, str]]) -> List[Dict[str, Any]]:
    output: List[Dict[str, Any]] = []
    for row in summary_rows:
        agent = row.get("agent_type", "")
        item: Dict[str, Any] = {
            "agent_type": agent,
            "episodes": row.get("episodes", ""),
            "success_rate_mean": row.get("success_rate_mean", ""),
        }
        for group, metrics in METRIC_GROUPS.items():
            values = [_metric_mean(row, metric) for metric in metrics if row.get(metric + "_mean") not in (None, "")]
            item[group + "_score"] = sum(values) if values else ""
            for metric in metrics:
                key = metric + "_mean"
                if key in row:
                    item[key] = row.get(key)
        output.append(item)
    return output


def build_reference_improvements(delta_summary: List[Dict[str, str]], reference_agent: str) -> List[Dict[str, Any]]:
    output: List[Dict[str, Any]] = []
    for row in delta_summary:
        agent = row.get("agent_type", "")
        item: Dict[str, Any] = {
            "reference_agent": reference_agent,
            "baseline_agent": agent,
            "paired_count": row.get("paired_count", ""),
        }
        for metric in sorted(LOWER_IS_BETTER):
            key = metric + "_delta_vs_ref_mean"
            ci_key = metric + "_delta_vs_ref_ci95"
            if key not in row:
                continue
            delta = _safe_float(row.get(key))
            item[key] = row.get(key)
            item[ci_key] = row.get(ci_key, "")
            item[metric + "_qwen_better"] = delta > 0.0
        output.append(item)
    return output


def build_stratified_findings(strata_rows: List[Dict[str, str]], reference_agent: str) -> List[Dict[str, Any]]:
    grouped: Dict[tuple, List[Dict[str, str]]] = {}
    for row in strata_rows:
        key = (row.get("factor", ""), row.get("level", ""))
        grouped.setdefault(key, []).append(row)
    output: List[Dict[str, Any]] = []
    for (factor, level), rows in sorted(grouped.items()):
        if not rows:
            continue
        best = min(rows, key=lambda row: _metric_mean(row, "objective_score"))
        ref = next((row for row in rows if row.get("agent_type") == reference_agent), None)
        item = {
            "factor": factor,
            "level": level,
            "best_agent": best.get("agent_type", ""),
            "best_objective_score_mean": best.get("objective_score_mean", ""),
            "reference_agent": reference_agent,
            "reference_objective_score_mean": ref.get("objective_score_mean", "") if ref else "",
            "reference_is_best": bool(ref and ref.get("agent_type") == best.get("agent_type")),
        }
        if ref:
            item["reference_gap_to_best"] = _metric_mean(ref, "objective_score") - _metric_mean(best, "objective_score")
        output.append(item)
    return output


def render_findings_md(
    metric_groups: List[Dict[str, Any]],
    improvements: List[Dict[str, Any]],
    strata: List[Dict[str, Any]],
    reproducibility: Dict[str, Any],
    reference_agent: str,
) -> str:
    lines = [
        "# TR-C CSV-First Experimental Analysis",
        "",
        "This analysis is generated from CSV/JSON report artifacts. It is evidence for manuscript writing, not the manuscript itself.",
        "",
        "## Evidence Coverage",
        "",
        "- Rows / agents / scenarios: `%s / %s / %s`" % (
            reproducibility.get("row_count", 0),
            reproducibility.get("agent_count", 0),
            reproducibility.get("scenario_count", 0),
        ),
        "- Background modes: `%s`" % ", ".join(reproducibility.get("background_modes", [])),
        "- Density labels: `%s`" % ", ".join(reproducibility.get("density_labels", [])),
        "- Seeds: `%s`" % ", ".join(reproducibility.get("seeds", [])),
        "",
        "## Reference Comparison",
        "",
    ]
    qwen_better_counts: Dict[str, int] = {}
    for row in improvements:
        for key, value in row.items():
            if key.endswith("_qwen_better") and value is True:
                qwen_better_counts[key.replace("_qwen_better", "")] = qwen_better_counts.get(key.replace("_qwen_better", ""), 0) + 1
    if qwen_better_counts:
        for metric, count in sorted(qwen_better_counts.items(), key=lambda item: (-item[1], item[0]))[:8]:
            lines.append("- `%s`: `%s` baselines worse than `%s` on paired mean delta." % (metric, count, reference_agent))
    else:
        lines.append("- No paired improvement claims are available yet.")
    lines.extend(["", "## Stratified Robustness", ""])
    ref_best = sum(1 for row in strata if row.get("reference_is_best"))
    lines.append("- `%s` is best in `%d / %d` factor-level strata by objective score." % (reference_agent, ref_best, len(strata)))
    lines.extend([
        "",
        "## Machine-Readable Outputs",
        "",
        "- `trc_metric_group_summary.csv`",
        "- `trc_reference_improvements.csv`",
        "- `trc_stratified_findings.csv`",
        "- `trc_analysis_manifest.json`",
    ])
    return "\n".join(lines) + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description="Build TR-C-oriented CSV-first analysis from ParkSim-VLA paper report outputs.")
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--reference-agent", default="qwen_vla")
    args = parser.parse_args()

    report_dir = args.report_dir.resolve()
    out_dir = args.out_dir.resolve() if args.out_dir else report_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = _load_csv(report_dir / "paper_summary.csv")
    delta_summary = _load_csv(report_dir / "paper_paired_delta_summary.csv")
    strata_rows = _load_csv(report_dir / "paper_stratified_summary.csv")
    reproducibility = _load_json(report_dir / "paper_reproducibility.json", {}) or {}

    metric_groups = build_metric_group_summary(summary_rows)
    improvements = build_reference_improvements(delta_summary, args.reference_agent)
    strata = build_stratified_findings(strata_rows, args.reference_agent)

    _write_csv(out_dir / "trc_metric_group_summary.csv", metric_groups)
    _write_csv(out_dir / "trc_reference_improvements.csv", improvements)
    _write_csv(out_dir / "trc_stratified_findings.csv", strata)
    (out_dir / "trc_key_findings.md").write_text(render_findings_md(metric_groups, improvements, strata, reproducibility, args.reference_agent))
    manifest = {
        "type": "parksim_vla_trc_csv_first_analysis",
        "report_dir": str(report_dir),
        "out_dir": str(out_dir),
        "reference_agent": args.reference_agent,
        "inputs": [
            "paper_summary.csv",
            "paper_paired_delta_summary.csv",
            "paper_stratified_summary.csv",
            "paper_reproducibility.json",
        ],
        "outputs": [
            "trc_metric_group_summary.csv",
            "trc_reference_improvements.csv",
            "trc_stratified_findings.csv",
            "trc_key_findings.md",
            "trc_analysis_manifest.json",
        ],
        "row_counts": {
            "metric_group_summary": len(metric_groups),
            "reference_improvements": len(improvements),
            "stratified_findings": len(strata),
        },
    }
    (out_dir / "trc_analysis_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print("trc_analysis=%s metric_groups=%d improvements=%d strata=%d" % (out_dir, len(metric_groups), len(improvements), len(strata)))


if __name__ == "__main__":
    main()
