import argparse
import csv
import json
import math
import pickle
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional

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


def collect_mode_metrics(experiment_dir: Path, mode: str) -> Dict[str, Any]:
    mode_dir = experiment_dir / mode
    log_dir = mode_dir / "logs"
    trace_path = _latest_vehicle_file(log_dir, "trace.jsonl") or (log_dir / "vehicle_1_trace.jsonl")
    summary_path = _latest_vehicle_file(log_dir, "summary.json") or (log_dir / "vehicle_1_summary.json")
    decisions_path = log_dir / "qwen_vla_decisions.jsonl"

    trace = _load_jsonl(trace_path)
    summary = _load_json(summary_path)
    decisions = _load_jsonl(decisions_path)

    speeds = [_safe_float(row.get("speed")) for row in trace]
    steering = [_safe_float(row.get("steering")) for row in trace]
    accelerations = [_safe_float(row.get("acceleration")) for row in trace]
    final = trace[-1] if trace else {}
    first = trace[0] if trace else {}

    qwen_latencies = [_safe_float(row.get("latency_seconds")) for row in decisions]
    applied_actions = [row.get("applied_action") or {} for row in decisions]
    selected_spots = [action.get("target_spot_index") for action in applied_actions if action.get("target_spot_index") is not None]
    action_types = [action.get("action_type") for action in applied_actions if action.get("action_type")]

    metrics = {
        "mode": mode,
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
        "qwen_latency_mean": _mean(qwen_latencies),
        "qwen_latency_max": max(qwen_latencies, default=0.0),
        "first_action_type": action_types[0] if action_types else None,
        "final_x": _safe_float((summary.get("final_state") or {}).get("x"), _safe_float(final.get("x"))),
        "final_y": _safe_float((summary.get("final_state") or {}).get("y"), _safe_float(final.get("y"))),
        "trace_path": str(trace_path),
        "summary_path": str(summary_path),
        "decisions_path": str(decisions_path) if decisions_path.exists() else "",
    }
    metrics.update(collect_safety_metrics(log_dir, trace_path, trace, decisions))
    return {"metrics": metrics, "trace": trace, "summary": summary, "decisions": decisions}


def write_metrics(experiment_dir: Path, collected: Dict[str, Dict[str, Any]]) -> None:
    rows = [collected[mode]["metrics"] for mode in MODES if mode in collected]
    with (experiment_dir / "metrics.json").open("w") as f:
        json.dump(rows, f, indent=2)
    fields = [
        "mode", "completed", "assigned_spot_index", "executed_spot_index", "selected_spot_index",
        "total_time", "total_non_idle_time", "path_length", "mean_speed", "max_speed",
        "idle_time", "low_speed_time", "brake_time", "waiting_time", "decision_count",
        "qwen_fallback_count", "shield_rejection_count", "qwen_latency_mean", "qwen_latency_max", "first_action_type",
        "other_vehicle_trace_count", "min_other_distance_m", "near_miss_event_count", "near_miss_time_s",
        "collision_proxy_event_count", "collision_proxy_time_s", "min_ttc_s", "unsafe_occupancy_action_count",
        "malformed_decision_count", "target_mismatch_decision_count", "missing_reason_code_count",
        "candidate_action_count_mean", "available_candidate_count_mean",
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
        ("qwen_latency_mean", "qwen_latency_mean_s"),
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
