import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

DEFAULT_METRICS = [
    "completed",
    "objective_score",
    "path_length",
    "total_non_idle_time",
    "idle_time",
    "near_miss_event_count",
    "collision_proxy_event_count",
    "unsafe_occupancy_action_count",
    "shield_rejection_count",
    "qwen_fallback_count",
    "qwen_latency_mean",
]


def _load_json(path: Path) -> Any:
    with path.open() as f:
        return json.load(f)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        if value is None or value == "":
            return default
        return float(value)
    except Exception:
        return default


def _mean(values: Iterable[float]) -> float:
    values = list(values)
    return sum(values) / len(values) if values else 0.0


def _std(values: Iterable[float]) -> float:
    values = list(values)
    if len(values) < 2:
        return 0.0
    m = _mean(values)
    return math.sqrt(sum((v - m) ** 2 for v in values) / (len(values) - 1))


def _ci95(values: Iterable[float]) -> float:
    values = list(values)
    if len(values) < 2:
        return 0.0
    return 1.96 * _std(values) / math.sqrt(len(values))


def collect_rows(inputs: List[Path], require_validation_ok: bool = True) -> Tuple[List[Dict[str, Any]], List[str]]:
    rows: List[Dict[str, Any]] = []
    warnings: List[str] = []
    for bench_dir in inputs:
        bench_dir = bench_dir.resolve()
        metrics_path = bench_dir / "metrics.json"
        if not metrics_path.exists():
            warnings.append("skip %s: missing metrics.json" % bench_dir)
            continue
        validation_path = bench_dir / "validation.json"
        if require_validation_ok:
            if not validation_path.exists():
                warnings.append("skip %s: missing validation.json" % bench_dir)
                continue
            validation = _load_json(validation_path)
            if not validation.get("ok"):
                warnings.append("skip %s: validation not ok" % bench_dir)
                continue
        metrics = _load_json(metrics_path)
        if not isinstance(metrics, list):
            warnings.append("skip %s: metrics.json is not a list" % bench_dir)
            continue
        run_config = {}
        run_config_path = bench_dir / "run_config.json"
        if run_config_path.exists():
            run_config = _load_json(run_config_path)
        for idx, row in enumerate(metrics):
            item = dict(row)
            item["benchmark_dir"] = str(bench_dir)
            item["benchmark_id"] = bench_dir.name
            item["row_index"] = idx
            item["pair_key"] = "%s/%s" % (bench_dir.name, item.get("scenario_id", "scenario"))
            item["git_commit"] = run_config.get("git_commit", "")
            item["git_branch"] = run_config.get("git_branch", "")
            rows.append(item)
    return rows, warnings


def summarize(rows: List[Dict[str, Any]], metrics: List[str]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("agent_type"))].append(row)
    summary: List[Dict[str, Any]] = []
    for agent in sorted(grouped):
        group = grouped[agent]
        item: Dict[str, Any] = {"agent_type": agent, "episodes": len(group)}
        item["scenario_count"] = len({row.get("pair_key") for row in group})
        item["success_rate_mean"] = _mean(1.0 if row.get("completed") else 0.0 for row in group)
        for metric in metrics:
            if metric == "completed":
                continue
            values = [_safe_float(row.get(metric)) for row in group]
            item[metric + "_mean"] = _mean(values)
            item[metric + "_std"] = _std(values)
            item[metric + "_ci95"] = _ci95(values)
        summary.append(item)
    return summary


def paired_deltas(rows: List[Dict[str, Any]], reference_agent: str, metrics: List[str]) -> List[Dict[str, Any]]:
    by_pair: Dict[str, Dict[str, Dict[str, Any]]] = defaultdict(dict)
    for row in rows:
        by_pair[str(row.get("pair_key"))][str(row.get("agent_type"))] = row
    deltas: List[Dict[str, Any]] = []
    for pair_key, agents in sorted(by_pair.items()):
        reference = agents.get(reference_agent)
        if not reference:
            continue
        for agent, row in sorted(agents.items()):
            if agent == reference_agent:
                continue
            item = {
                "pair_key": pair_key,
                "agent_type": agent,
                "reference_agent": reference_agent,
                "scenario_id": row.get("scenario_id"),
                "benchmark_id": row.get("benchmark_id"),
            }
            for metric in metrics:
                if metric == "completed":
                    continue
                item[metric + "_delta_vs_ref"] = _safe_float(row.get(metric)) - _safe_float(reference.get(metric))
            deltas.append(item)
    return deltas


def summarize_deltas(deltas: List[Dict[str, Any]], metrics: List[str]) -> List[Dict[str, Any]]:
    grouped: Dict[str, List[Dict[str, Any]]] = defaultdict(list)
    for row in deltas:
        grouped[str(row.get("agent_type"))].append(row)
    summary: List[Dict[str, Any]] = []
    for agent in sorted(grouped):
        group = grouped[agent]
        item: Dict[str, Any] = {"agent_type": agent, "paired_count": len(group)}
        for metric in metrics:
            if metric == "completed":
                continue
            key = metric + "_delta_vs_ref"
            values = [_safe_float(row.get(key)) for row in group]
            item[key + "_mean"] = _mean(values)
            item[key + "_std"] = _std(values)
            item[key + "_ci95"] = _ci95(values)
        summary.append(item)
    return summary


def write_csv(path: Path, rows: List[Dict[str, Any]]) -> None:
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


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        json.dump(payload, f, indent=2)


def write_markdown(path: Path, summary: List[Dict[str, Any]], delta_summary: List[Dict[str, Any]], reference_agent: str) -> None:
    lines = [
        "# ParkSim-VLA Paper Report",
        "",
        "## Agent Summary",
        "",
        "| agent | episodes | success | objective | path_m | non_idle_s | near_miss | collision_proxy | unsafe_spot | latency_s |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in summary:
        lines.append(
            "| {agent_type} | {episodes:d} | {success_rate_mean:.3f} | {objective_score_mean:.3f} +/- {objective_score_ci95:.3f} | "
            "{path_length_mean:.3f} +/- {path_length_ci95:.3f} | {total_non_idle_time_mean:.3f} +/- {total_non_idle_time_ci95:.3f} | "
            "{near_miss_event_count_mean:.3f} | {collision_proxy_event_count_mean:.3f} | {unsafe_occupancy_action_count_mean:.3f} | "
            "{qwen_latency_mean_mean:.3f} |".format(**_with_defaults(row))
        )
    lines.extend([
        "",
        "## Paired Delta vs `%s`" % reference_agent,
        "",
        "Positive delta means the agent is worse than the reference for lower-is-better metrics.",
        "",
        "| agent | pairs | objective_delta | path_delta | non_idle_delta | near_miss_delta | collision_delta | unsafe_spot_delta |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ])
    for row in delta_summary:
        lines.append(
            "| {agent_type} | {paired_count:d} | {objective_score_delta_vs_ref_mean:.3f} +/- {objective_score_delta_vs_ref_ci95:.3f} | "
            "{path_length_delta_vs_ref_mean:.3f} | {total_non_idle_time_delta_vs_ref_mean:.3f} | "
            "{near_miss_event_count_delta_vs_ref_mean:.3f} | {collision_proxy_event_count_delta_vs_ref_mean:.3f} | "
            "{unsafe_occupancy_action_count_delta_vs_ref_mean:.3f} |".format(**_with_defaults(row))
        )
    path.write_text("\n".join(lines) + "\n")


def write_latex(path: Path, summary: List[Dict[str, Any]]) -> None:
    line_end = "\\\\"
    lines = [
        "\\begin{tabular}{lrrrrr}",
        "\\toprule",
        "Agent & Success & Objective & Path (m) & Non-idle (s) & Safety events " + line_end,
        "\\midrule",
    ]
    for row in summary:
        data = _with_defaults(row)
        safety = data["near_miss_event_count_mean"] + data["collision_proxy_event_count_mean"] + data["unsafe_occupancy_action_count_mean"]
        lines.append(
            "{} & {:.3f} & {:.3f} +/- {:.3f} & {:.3f} +/- {:.3f} & {:.3f} +/- {:.3f} & {:.3f} {}".format(
                data["agent_type"].replace("_", "\\_"),
                data["success_rate_mean"],
                data["objective_score_mean"],
                data["objective_score_ci95"],
                data["path_length_mean"],
                data["path_length_ci95"],
                data["total_non_idle_time_mean"],
                data["total_non_idle_time_ci95"],
                safety,
                line_end,
            )
        )
    lines.extend(["\\bottomrule", "\\end{tabular}"])
    path.write_text("\n".join(lines) + "\n")

def _with_defaults(row: Dict[str, Any]) -> Dict[str, Any]:
    defaults = defaultdict(float)
    defaults.update(row)
    defaults.setdefault("agent_type", "")
    defaults.setdefault("episodes", 0)
    defaults.setdefault("paired_count", 0)
    return defaults


def main() -> None:
    parser = argparse.ArgumentParser(description="Build paper-ready aggregate tables from ParkSim-VLA benchmark directories.")
    parser.add_argument("out_dir", type=Path)
    parser.add_argument("--inputs", nargs="+", type=Path, required=True)
    parser.add_argument("--reference-agent", default="qwen_vla")
    parser.add_argument("--allow-invalid", action="store_true")
    parser.add_argument("--metrics", default=",".join(DEFAULT_METRICS))
    args = parser.parse_args()

    out_dir = args.out_dir.resolve()
    metrics = [item for item in args.metrics.split(",") if item]
    rows, warnings = collect_rows(args.inputs, require_validation_ok=not args.allow_invalid)
    summary = summarize(rows, metrics)
    deltas = paired_deltas(rows, args.reference_agent, metrics)
    delta_summary = summarize_deltas(deltas, metrics)
    out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(out_dir / "paper_rows.csv", rows)
    write_csv(out_dir / "paper_summary.csv", summary)
    write_csv(out_dir / "paper_paired_deltas.csv", deltas)
    write_csv(out_dir / "paper_paired_delta_summary.csv", delta_summary)
    write_markdown(out_dir / "paper_summary.md", summary, delta_summary, args.reference_agent)
    write_latex(out_dir / "paper_table.tex", summary)
    manifest = {
        "type": "parksim_vla_paper_report",
        "out_dir": str(out_dir),
        "inputs": [str(path.resolve()) for path in args.inputs],
        "reference_agent": args.reference_agent,
        "metrics": metrics,
        "row_count": len(rows),
        "agent_count": len(summary),
        "paired_delta_count": len(deltas),
        "warnings": warnings,
    }
    write_json(out_dir / "paper_report_manifest.json", manifest)
    for warning in warnings:
        print("warning: %s" % warning)
    print("paper_report=%s rows=%d agents=%d paired=%d" % (out_dir, len(rows), len(summary), len(deltas)))


if __name__ == "__main__":
    main()
