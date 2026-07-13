import json
import tempfile
from pathlib import Path
from typing import Any, Dict, Iterable

from parksim.vla.benchmark import objective_score
from parksim.vla.compare_results import collect_mode_metrics
from parksim.vla.paper_gate import (
    REQUIRED_METRIC_COLUMNS,
    TRC_METRIC_GROUPS,
    TRC_PRIMARY_TEST_METRICS,
    _required_metric_value_complete,
)
from parksim.vla.paper_report import DEFAULT_METRICS, MARKDOWN_TEST_METRICS
from parksim.vla.safety_metrics import collect_system_traffic_metrics
from parksim.vla.trc_analysis import METRIC_CATALOG


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload))


def _write_jsonl(path: Path, rows: Iterable[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def _trace_row(vehicle_id: int, agent_type: str, sim_time: float, wall_time: float, x: float) -> Dict[str, Any]:
    return {
        "vehicle_id": vehicle_id,
        "spawn_event_id": "controlled_ego",
        "vehicle_role": "controlled_ego",
        "agent_type": agent_type,
        "is_controlled_ego": True,
        "intent_observable": True,
        "sim_time": sim_time,
        "time": sim_time,
        "wall_time": wall_time,
        "simulation_step_seconds": 1.0,
        "simulation_speedup": 5.0,
        "wall_timer_period_seconds": 0.2,
        "x": x,
        "y": 0.0,
        "speed": 0.2,
        "steering": 0.0,
        "acceleration": 0.0,
        "task": "CRUISE",
        "waiting_for": 0,
        "is_braking": False,
        "spot_index": 7,
        "vehicle_spot_index": 7,
        "is_final": sim_time >= 1.0,
    }


def _build_metric_fixture(root: Path, mode: str, synchronous: bool) -> Dict[str, Any]:
    mode_dir = root / mode
    log_dir = mode_dir / "logs"
    trace = [
        _trace_row(1, mode, 0.0, 100.0, 0.0),
        _trace_row(1, mode, 1.0, 101.0, 0.2),
    ]
    _write_jsonl(log_dir / "vehicle_1_trace.jsonl", trace)
    _write_json(log_dir / "vehicle_1_summary.json", {
        "vehicle_id": 1,
        "vehicle_role": "controlled_ego",
        "agent_type": mode,
        "is_controlled_ego": True,
        "intent_observable": True,
        "completed": True,
        "censored": False,
        "spot_index": 7,
        "vehicle_spot_index": 7,
        "total_time": 1.0,
        "total_non_idle_time": 1.0,
        "final_state": {"x": 0.2, "y": 0.0},
    })
    _write_json(log_dir / "traffic_schedule.json", {
        "long_horizon_duration": 3600.0,
        "events": [],
    })
    _write_jsonl(log_dir / "traffic_events.jsonl", [])

    if synchronous:
        _write_jsonl(log_dir / "qwen_vla_decisions.jsonl", [{
            "decision": {
                "action_id": "wait-1",
                "reason_code": "YIELD_WAIT",
                "used_fallback": False,
            },
            "context": {
                "valid_actions": [{
                    "action_id": "wait-1",
                    "action_type": "WAIT",
                    "features": {"selectable": True},
                }],
            },
            "applied_action": {
                "action_id": "wait-1",
                "action_type": "WAIT",
                "features": {"selectable": True},
            },
            "shield_reason": "ok",
            "latency_seconds": 999.0,
        }])
        _write_jsonl(mode_dir / "fleet_epochs.jsonl", [{
            "run_id": "metric-contract",
            "epoch_id": 1,
            "sim_time": 0.5,
            "policy_mode": "direct",
            "model_id": "contract-model",
            "model_revision": "contract-revision",
            "simulation_time_policy": "decision_then_advance",
            "wall_clock_latency_in_performance_metrics": False,
            "decision_barrier_status": "applied",
            "collected_vehicle_ids": [1],
            "fleet_decisions": [{"vehicle_id": 1, "action_id": "wait-1"}],
            "missing_vehicle_ids": [],
            "decision_ack_expected_vehicle_ids": [1],
            "decision_ack_received_vehicle_ids": [1],
            "decision_acknowledgements": [{
                "vehicle_id": 1,
                "applied": True,
                "decision_sim_time": 0.5,
            }],
            "pre_feedback_critique": {
                "hard_violation_count": 0,
                "quality_warning_count": 0,
                "route_conflict_count": 0,
            },
            "post_feedback_critique": {
                "hard_violation_count": 0,
                "quality_warning_count": 0,
                "route_conflict_count": 0,
            },
            "fleet_context": {
                "protocol_version": "ParkSim-Qwen-VLA-Fleet-Decision-v2",
                "prompt_version": "metric-contract-v1",
            },
            "latency_seconds": 999.0,
            "batch_wait_seconds": 999.0,
        }])

    metrics = collect_mode_metrics(root, mode)["metrics"]
    metrics["objective_score"] = objective_score(metrics)
    incomplete = [
        metric
        for metric in REQUIRED_METRIC_COLUMNS
        if not _required_metric_value_complete(metric, metrics.get(metric))
    ]
    assert not incomplete, "%s required metrics incomplete: %s" % (mode, incomplete)
    return metrics


def _assert_sim_time_alignment(root: Path) -> None:
    log_dir = root / "alignment"
    for vehicle_id, wall_offset, y in ((1, 100.0, 0.0), (2, 1000.0, 1.0)):
        agent = "rule_based"
        rows = []
        for sim_time in (0.0, 1.0):
            row = _trace_row(vehicle_id, agent, sim_time, wall_offset + sim_time, sim_time * 0.2)
            row.update({
                "spawn_event_id": "vehicle-%d" % vehicle_id,
                "vehicle_role": "human_rule_entering",
                "is_controlled_ego": False,
                "y": y,
            })
            rows.append(row)
        _write_jsonl(log_dir / ("vehicle_%d_trace.jsonl" % vehicle_id), rows)
        _write_json(log_dir / ("vehicle_%d_summary.json" % vehicle_id), {
            "vehicle_id": vehicle_id,
            "vehicle_role": "human_rule_entering",
            "agent_type": agent,
            "completed": True,
            "total_time": 1.0,
        })
    _write_json(log_dir / "traffic_schedule.json", {"long_horizon_duration": 3600.0, "events": []})
    metrics = collect_system_traffic_metrics(log_dir)
    assert metrics["system_near_miss_event_count"] == 1, metrics
    assert metrics["system_collision_proxy_event_count"] == 1, metrics


def main() -> None:
    with tempfile.TemporaryDirectory(prefix="parksim-metric-contract-") as raw_root:
        root = Path(raw_root)
        baseline = _build_metric_fixture(root / "baseline", "rule_based", synchronous=False)
        qwen = _build_metric_fixture(root / "qwen", "mllm_direct", synchronous=True)
        assert baseline["simulation_time_policy"] == "not_applicable"
        assert baseline["simulation_step_seconds"] == 1.0
        assert baseline["simulation_speedup"] == 5.0
        assert qwen["simulation_time_policy"] == "decision_then_advance"
        assert qwen["decision_barrier_ack_coverage_rate"] == 1.0
        assert qwen["decision_barrier_failure_count"] == 0
        assert qwen["audit_qwen_wall_latency_mean"] == 999.0
        assert qwen["wall_clock_latency_in_performance_metrics"] is False
        assert qwen["trace_sim_step_gap_count"] == 0
        _assert_sim_time_alignment(root)

    paper_metrics = set(DEFAULT_METRICS) | set(MARKDOWN_TEST_METRICS)
    paper_metrics.update(TRC_PRIMARY_TEST_METRICS)
    for group_metrics in TRC_METRIC_GROUPS.values():
        paper_metrics.update(group_metrics)
    paper_metrics.update(item["metric"] for item in METRIC_CATALOG)
    assert not [metric for metric in paper_metrics if "latency" in metric.lower()]
    print("metric contract smoke passed: simulator-time alignment and latency exclusion verified")


if __name__ == "__main__":
    main()
