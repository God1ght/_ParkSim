import argparse
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


def _load_json(path: Path) -> Any:
    with path.open() as handle:
        return json.load(handle)


def _metric_rows(benchmark_dir: Path) -> Dict[str, Dict[str, Any]]:
    validation = _load_json(benchmark_dir / "validation.json")
    if validation.get("ok") is not True:
        raise ValueError("benchmark validation failed: %s" % benchmark_dir)
    rows = _load_json(benchmark_dir / "metrics.json")
    if not isinstance(rows, list) or not rows:
        raise ValueError("expected metric rows in %s" % benchmark_dir)
    indexed: Dict[str, Dict[str, Any]] = {}
    for row in rows:
        agent = str(row.get("agent_type", "")).strip()
        if not agent:
            raise ValueError("metric row has no agent_type in %s" % benchmark_dir)
        if agent in indexed:
            raise ValueError("duplicate agent_type %s in %s" % (agent, benchmark_dir))
        indexed[agent] = row
    return indexed


def _float(row: Dict[str, Any], key: str) -> float:
    value = float(row.get(key))
    if not math.isfinite(value):
        raise ValueError("nonfinite %s=%r" % (key, row.get(key)))
    return value


def _relative_delta(left: float, right: float) -> float:
    return abs(right - left) / max(1.0, abs(left))


def _same_numeric(left: Dict[str, Any], right: Dict[str, Any], metrics: Iterable[str]) -> bool:
    return all(_float(left, metric) == _float(right, metric) for metric in metrics)


def _numeric_pairs(
    left: Dict[str, Any], right: Dict[str, Any], metrics: Iterable[str]
) -> Dict[str, List[Any]]:
    return {metric: [left.get(metric), right.get(metric)] for metric in metrics}


def compare(
    baseline_dir: Path,
    accelerated_dir: Path,
    simulation_step: float,
    accelerated_speedup: float,
    expected_agents: Optional[List[str]] = None,
) -> Dict[str, Any]:
    baseline_rows = _metric_rows(baseline_dir)
    accelerated_rows = _metric_rows(accelerated_dir)
    checks: List[Dict[str, Any]] = []

    def require(name: str, ok: bool, detail: Dict[str, Any]) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    baseline_agents = set(baseline_rows)
    accelerated_agents = set(accelerated_rows)
    required_agents = set(expected_agents or baseline_rows)
    require(
        "agent_set",
        baseline_agents == accelerated_agents == required_agents,
        {
            "expected": sorted(required_agents),
            "baseline": sorted(baseline_agents),
            "accelerated": sorted(accelerated_agents),
        },
    )

    comparable_agents = sorted(baseline_agents & accelerated_agents & required_agents)
    exact_traffic_metrics = (
        "traffic_scheduled_count",
        "traffic_spawned_count",
        "traffic_delayed_count",
        "traffic_skipped_count",
        "total_vehicle_trace_count",
        "completed_vehicle_count",
        "automated_demand_released_count",
        "automated_demand_completed_count",
        "automated_demand_backlog_count",
    )
    exact_safety_metrics = (
        "system_near_miss_event_count",
        "system_collision_proxy_event_count",
        "trajectory_conflict_event_count",
        "mixed_intent_conflict_event_count",
    )
    exact_barrier_metrics = (
        "decision_barrier_epoch_count",
        "decision_barrier_complete_count",
        "decision_barrier_failure_count",
        "decision_barrier_ack_expected_count",
        "decision_barrier_ack_received_count",
        "decision_barrier_sim_time_mismatch_count",
        "cloud_fleet_decision_count",
        "cloud_fleet_vehicle_decision_count",
        "cloud_fleet_missing_vehicle_decision_count",
    )
    fleet_continuous_metrics = (
        "fleet_mean_automated_path_length",
        "fleet_mean_automated_waiting_time",
        "fleet_mean_automated_total_time",
    )

    for agent in comparable_agents:
        baseline = baseline_rows[agent]
        accelerated = accelerated_rows[agent]
        prefix = "%s:" % agent
        require(
            prefix + "simulation_step",
            abs(_float(baseline, "simulation_step_seconds") - simulation_step) <= 1e-9
            and abs(_float(accelerated, "simulation_step_seconds") - simulation_step) <= 1e-9,
            {
                "expected": simulation_step,
                "baseline": baseline.get("simulation_step_seconds"),
                "accelerated": accelerated.get("simulation_step_seconds"),
            },
        )
        require(
            prefix + "speedup_identity",
            abs(_float(baseline, "simulation_speedup") - 1.0) <= 1e-9
            and abs(_float(accelerated, "simulation_speedup") - accelerated_speedup) <= 1e-9,
            {
                "baseline": baseline.get("simulation_speedup"),
                "accelerated": accelerated.get("simulation_speedup"),
            },
        )
        require(
            prefix + "no_simulation_step_gaps",
            _float(baseline, "trace_sim_step_gap_count") == 0.0
            and _float(accelerated, "trace_sim_step_gap_count") == 0.0,
            _numeric_pairs(baseline, accelerated, ("trace_sim_step_gap_count",)),
        )
        require(
            prefix + "trace_integrity",
            baseline.get("trace_integrity_ok") is True and accelerated.get("trace_integrity_ok") is True,
            {
                "baseline": baseline.get("trace_integrity_ok"),
                "accelerated": accelerated.get("trace_integrity_ok"),
            },
        )
        require(
            prefix + "paired_traffic_schedule",
            bool(baseline.get("traffic_schedule_hash"))
            and baseline.get("traffic_schedule_hash") == accelerated.get("traffic_schedule_hash"),
            {
                "baseline": baseline.get("traffic_schedule_hash"),
                "accelerated": accelerated.get("traffic_schedule_hash"),
            },
        )
        require(
            prefix + "traffic_event_equivalence",
            _same_numeric(baseline, accelerated, exact_traffic_metrics),
            _numeric_pairs(baseline, accelerated, exact_traffic_metrics),
        )
        require(
            prefix + "task_outcome",
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
            prefix + "path_length_equivalence",
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
            prefix + "final_position_equivalence",
            final_distance <= 0.5,
            {"tolerance_m": 0.5, "distance_m": final_distance},
        )
        require(
            prefix + "fleet_continuous_equivalence",
            all(
                _relative_delta(_float(baseline, metric), _float(accelerated, metric)) <= 0.02
                for metric in fleet_continuous_metrics
            ),
            {
                metric: {
                    "baseline": baseline.get(metric),
                    "accelerated": accelerated.get(metric),
                    "relative_delta": _relative_delta(_float(baseline, metric), _float(accelerated, metric)),
                }
                for metric in fleet_continuous_metrics
            },
        )
        require(
            prefix + "safety_event_equivalence",
            _same_numeric(baseline, accelerated, exact_safety_metrics),
            _numeric_pairs(baseline, accelerated, exact_safety_metrics),
        )
        synchronous_agent = agent in {
            "qwen_vla",
            "mllm_direct",
            "mllm_self_reflect",
            "mllm_external_feedback",
            "fleet_min_cost",
        }
        if synchronous_agent:
            require(
                prefix + "barrier_count_equivalence",
                _same_numeric(baseline, accelerated, exact_barrier_metrics),
                _numeric_pairs(baseline, accelerated, exact_barrier_metrics),
            )
            require(
                prefix + "synchronous_barrier_integrity",
                _float(baseline, "decision_barrier_epoch_count") > 0.0
                and _float(accelerated, "decision_barrier_epoch_count") > 0.0
                and _float(baseline, "decision_barrier_ack_coverage_rate") == 1.0
                and _float(accelerated, "decision_barrier_ack_coverage_rate") == 1.0
                and _float(baseline, "decision_barrier_failure_count") == 0.0
                and _float(accelerated, "decision_barrier_failure_count") == 0.0
                and _float(baseline, "decision_barrier_sim_time_mismatch_count") == 0.0
                and _float(accelerated, "decision_barrier_sim_time_mismatch_count") == 0.0
                and baseline.get("simulation_time_policy") == "decision_then_advance"
                and accelerated.get("simulation_time_policy") == "decision_then_advance"
                and baseline.get("wall_clock_latency_in_performance_metrics") is False
                and accelerated.get("wall_clock_latency_in_performance_metrics") is False,
                {
                    "baseline_epoch_count": baseline.get("decision_barrier_epoch_count"),
                    "accelerated_epoch_count": accelerated.get("decision_barrier_epoch_count"),
                    "baseline_ack_coverage": baseline.get("decision_barrier_ack_coverage_rate"),
                    "accelerated_ack_coverage": accelerated.get("decision_barrier_ack_coverage_rate"),
                    "baseline_time_policy": baseline.get("simulation_time_policy"),
                    "accelerated_time_policy": accelerated.get("simulation_time_policy"),
                },
            )
    result = {
        "ok": all(item["ok"] for item in checks),
        "baseline_dir": str(baseline_dir),
        "accelerated_dir": str(accelerated_dir),
        "simulation_step_seconds": simulation_step,
        "accelerated_speedup": accelerated_speedup,
        "agents": comparable_agents,
        "checks": checks,
    }
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate 1x versus accelerated ParkSim dynamics equivalence.")
    parser.add_argument("baseline_dir", type=Path)
    parser.add_argument("accelerated_dir", type=Path)
    parser.add_argument("--simulation-step", type=float, default=0.1)
    parser.add_argument("--accelerated-speedup", type=float, default=5.0)
    parser.add_argument("--expected-agents", default="")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    expected_agents = [item.strip() for item in args.expected_agents.split(",") if item.strip()]
    result = compare(
        args.baseline_dir.resolve(),
        args.accelerated_dir.resolve(),
        simulation_step=float(args.simulation_step),
        accelerated_speedup=float(args.accelerated_speedup),
        expected_agents=expected_agents or None,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print("speedup_equivalence_ok=%s" % result["ok"])
    print("speedup_equivalence_report=%s" % args.output.resolve())
    if not result["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
