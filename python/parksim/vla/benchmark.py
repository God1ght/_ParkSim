import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List

from parksim.vla.compare_results import collect_mode_metrics


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


def objective_score(metrics: Dict[str, Any]) -> float:
    incomplete_penalty = 0.0 if metrics.get("completed") else 1000.0
    return (
        incomplete_penalty
        + _safe_float(metrics.get("path_length"))
        + 0.2 * _safe_float(metrics.get("total_non_idle_time"))
        + 0.1 * _safe_float(metrics.get("idle_time"))
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
    preferred = [
        "scenario_id", "agent_type", "seed", "background_mode", "spot_index",
        "completed", "objective_score", "total_time", "total_non_idle_time",
        "path_length", "idle_time", "low_speed_time", "waiting_time", "decision_count",
        "qwen_fallback_count", "shield_rejection_count", "qwen_latency_mean",
        "selected_spot_index", "executed_spot_index", "first_action_type",
        "trace_path", "summary_path", "decisions_path",
    ]
    extra = sorted(key for row in rows for key in row if key not in preferred)
    fields = preferred + extra
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
            "mean_objective_score": _mean(_safe_float(row.get("objective_score")) for row in group),
        })
    return summary


def write_summary(out_dir: Path, rows: List[Dict[str, Any]]) -> None:
    grouped = aggregate_by_agent(rows)
    _write_json(out_dir / "summary.json", grouped)
    lines = [
        "# ParkSim-VLA-Bench Summary",
        "",
        "| agent | episodes | success | objective | path_m | non_idle_s | idle_s | decisions | shield_rejects | latency_s |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in grouped:
        lines.append(
            "| {agent_type} | {episodes:d} | {success_rate:.3f} | {mean_objective_score:.3f} | "
            "{mean_path_length:.3f} | {mean_total_non_idle_time:.3f} | {mean_idle_time:.3f} | "
            "{mean_decision_count:.3f} | {mean_shield_rejection_count:.3f} | {mean_latency_s:.3f} |".format(**row)
        )
    lines.extend([
        "",
        "Objective score is lower-is-better: path length + time penalties + shield/fallback penalties + incomplete penalty.",
        "Use `metrics.csv` for statistical tests and `preference_dataset.jsonl` for Qwen policy optimization data.",
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
                    "objective": "minimize incomplete-penalized path/time/shield/fallback score",
                    "chosen": _preference_view(chosen),
                    "rejected": _preference_view(rejected),
                }
                f.write(json.dumps(record) + "\n")


def _preference_view(row: Dict[str, Any]) -> Dict[str, Any]:
    keys = [
        "completed", "objective_score", "path_length", "total_non_idle_time",
        "idle_time", "decision_count", "qwen_fallback_count", "shield_rejection_count",
        "selected_spot_index", "executed_spot_index", "first_action_type",
        "decisions_path", "trace_path",
    ]
    return {key: row.get(key) for key in keys}


def main() -> None:
    parser = argparse.ArgumentParser(description="Collect reproducible ParkSim VLA benchmark metrics.")
    sub = parser.add_subparsers(dest="cmd", required=True)
    collect_parser = sub.add_parser("collect")
    collect_parser.add_argument("out_dir", type=Path)
    collect_parser.add_argument("--root", type=Path, default=Path.cwd())
    args = parser.parse_args()

    if args.cmd == "collect":
        out_dir = args.out_dir.resolve()
        root = args.root.resolve()
        rows = collect(out_dir, root)
        write_metrics(out_dir, rows)
        write_summary(out_dir, rows)
        write_preference_dataset(out_dir, rows)
        print("benchmark metrics written to %s" % out_dir)
        for row in aggregate_by_agent(rows):
            print("{agent_type} episodes={episodes} success={success_rate:.3f} objective={mean_objective_score:.3f}".format(**row))


if __name__ == "__main__":
    main()
