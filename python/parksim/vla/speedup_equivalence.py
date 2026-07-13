import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, List


def _load_json(path: Path) -> Any:
    with path.open() as handle:
        return json.load(handle)


def _metric_row(benchmark_dir: Path) -> Dict[str, Any]:
    validation = _load_json(benchmark_dir / "validation.json")
    if validation.get("ok") is not True:
        raise ValueError("benchmark validation failed: %s" % benchmark_dir)
    rows = _load_json(benchmark_dir / "metrics.json")
    if not isinstance(rows, list) or len(rows) != 1:
        raise ValueError("expected one metric row in %s" % benchmark_dir)
    return rows[0]


def _float(row: Dict[str, Any], key: str) -> float:
    value = float(row.get(key))
    if not math.isfinite(value):
        raise ValueError("nonfinite %s=%r" % (key, row.get(key)))
    return value


def _relative_delta(left: float, right: float) -> float:
    return abs(right - left) / max(1.0, abs(left))


def compare(
    baseline_dir: Path,
    accelerated_dir: Path,
    simulation_step: float,
    accelerated_speedup: float,
) -> Dict[str, Any]:
    baseline = _metric_row(baseline_dir)
    accelerated = _metric_row(accelerated_dir)
    checks: List[Dict[str, Any]] = []

    def require(name: str, ok: bool, detail: Dict[str, Any]) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    require(
        "simulation_step",
        abs(_float(baseline, "simulation_step_seconds") - simulation_step) <= 1e-9
        and abs(_float(accelerated, "simulation_step_seconds") - simulation_step) <= 1e-9,
        {
            "expected": simulation_step,
            "baseline": baseline.get("simulation_step_seconds"),
            "accelerated": accelerated.get("simulation_step_seconds"),
        },
    )
    require(
        "speedup_identity",
        abs(_float(baseline, "simulation_speedup") - 1.0) <= 1e-9
        and abs(_float(accelerated, "simulation_speedup") - accelerated_speedup) <= 1e-9,
        {
            "baseline": baseline.get("simulation_speedup"),
            "accelerated": accelerated.get("simulation_speedup"),
        },
    )
    require(
        "no_simulation_step_gaps",
        _float(baseline, "trace_sim_step_gap_count") == 0.0
        and _float(accelerated, "trace_sim_step_gap_count") == 0.0,
        {
            "baseline": baseline.get("trace_sim_step_gap_count"),
            "accelerated": accelerated.get("trace_sim_step_gap_count"),
        },
    )
    require(
        "trace_integrity",
        baseline.get("trace_integrity_ok") is True and accelerated.get("trace_integrity_ok") is True,
        {
            "baseline": baseline.get("trace_integrity_ok"),
            "accelerated": accelerated.get("trace_integrity_ok"),
        },
    )
    require(
        "paired_traffic_schedule",
        bool(baseline.get("traffic_schedule_hash"))
        and baseline.get("traffic_schedule_hash") == accelerated.get("traffic_schedule_hash"),
        {
            "baseline": baseline.get("traffic_schedule_hash"),
            "accelerated": accelerated.get("traffic_schedule_hash"),
        },
    )
    require(
        "task_outcome",
        baseline.get("completed") == accelerated.get("completed")
        and baseline.get("executed_spot_index") == accelerated.get("executed_spot_index"),
        {
            "baseline_completed": baseline.get("completed"),
            "accelerated_completed": accelerated.get("completed"),
            "baseline_spot": baseline.get("executed_spot_index"),
            "accelerated_spot": accelerated.get("executed_spot_index"),
        },
    )
    baseline_path = _float(baseline, "path_length")
    accelerated_path = _float(accelerated, "path_length")
    require(
        "path_length_equivalence",
        _relative_delta(baseline_path, accelerated_path) <= 0.02,
        {
            "relative_tolerance": 0.02,
            "baseline": baseline_path,
            "accelerated": accelerated_path,
            "relative_delta": _relative_delta(baseline_path, accelerated_path),
        },
    )
    final_distance = math.hypot(
        _float(accelerated, "final_x") - _float(baseline, "final_x"),
        _float(accelerated, "final_y") - _float(baseline, "final_y"),
    )
    require(
        "final_position_equivalence",
        final_distance <= 0.5,
        {"tolerance_m": 0.5, "distance_m": final_distance},
    )
    require(
        "safety_event_equivalence",
        all(
            _float(baseline, metric) == _float(accelerated, metric)
            for metric in (
                "system_near_miss_event_count",
                "system_collision_proxy_event_count",
                "trajectory_conflict_event_count",
            )
        ),
        {
            metric: [baseline.get(metric), accelerated.get(metric)]
            for metric in (
                "system_near_miss_event_count",
                "system_collision_proxy_event_count",
                "trajectory_conflict_event_count",
            )
        },
    )
    result = {
        "ok": all(item["ok"] for item in checks),
        "baseline_dir": str(baseline_dir),
        "accelerated_dir": str(accelerated_dir),
        "simulation_step_seconds": simulation_step,
        "accelerated_speedup": accelerated_speedup,
        "checks": checks,
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate 1x versus accelerated ParkSim dynamics equivalence.")
    parser.add_argument("baseline_dir", type=Path)
    parser.add_argument("accelerated_dir", type=Path)
    parser.add_argument("--simulation-step", type=float, default=0.1)
    parser.add_argument("--accelerated-speedup", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = compare(
        args.baseline_dir.resolve(),
        args.accelerated_dir.resolve(),
        simulation_step=float(args.simulation_step),
        accelerated_speedup=float(args.accelerated_speedup),
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print("speedup_equivalence_ok=%s" % result["ok"])
    print("speedup_equivalence_report=%s" % args.output.resolve())
    if not result["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
