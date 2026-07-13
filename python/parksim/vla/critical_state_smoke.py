import json
import tempfile
from pathlib import Path

from parksim.vla.critical_state_analysis import (
    _critical_events,
    _critical_traffic_events,
    _critical_trajectory_intervals,
    _discover_epoch_paths,
    _write_summary,
)


def _write_jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def main():
    with tempfile.TemporaryDirectory(prefix="parksim-critical-state-") as tmp:
        suite = Path(tmp) / "suite"
        benchmark = suite / "benchmarks" / "mixed_medium"
        run_dir = (
            benchmark
            / "episodes"
            / "mixed_seed3_spot7_human_enter2_human_exit1_av_enter2_av_exit1"
            / "mllm_external_feedback"
        )
        suite.mkdir(parents=True)
        (suite / "benchmarks.jsonl").write_text(json.dumps({
            "benchmark_dir": str(benchmark),
            "status": "completed",
        }) + "\n")

        decisions = [
            {
                "vehicle_id": 1,
                "action_id": "wait_2s",
                "target_spot_index": None,
                "used_fallback": True,
                "shield_ok": False,
                "reason": "duplicate target fallback",
                "action_bundle": {"features": {"conflict_risk": 0.9, "conflict_vehicle_count": 3}},
            },
            {
                "vehicle_id": 2,
                "action_id": "wait_5s",
                "target_spot_index": None,
                "used_fallback": False,
                "shield_ok": True,
                "reason": "yield",
                "action_bundle": {"features": {"conflict_risk": 0.1, "conflict_vehicle_count": 1}},
            },
        ]
        _write_jsonl(run_dir / "fleet_epochs.jsonl", [{
            "run_id": "synthetic",
            "epoch_id": 4,
            "sim_time": 10.0,
            "trigger_type": "event",
            "trigger_reasons": {"1": "stalled_or_yielding_replan"},
            "fleet_decisions": decisions,
            "repair_attempted": True,
            "pre_feedback_critique": {
                "hard_violation_count": 2,
                "route_conflict_count": 1,
                "duplicate_target_count": 1,
            },
            "post_feedback_critique": {
                "hard_violation_count": 0,
                "route_conflict_count": 0,
                "duplicate_target_count": 0,
            },
            "fleet_context": {"state": {"human_intent_beliefs": [{"normalized_entropy": 0.9}]}},
            "decision_barrier_status": "applied",
            "fleet_bev_path": str(run_dir / "fleet_bev" / "fleet_epoch_00004.png"),
        }])
        _write_jsonl(run_dir / "logs" / "traffic_events.jsonl", [
            {
                "event_id": "enter_001",
                "sim_time": 9.0,
                "event_type": "entering",
                "status": "delayed",
                "reason": "entrance_occupied",
                "intent_observable": False,
                "vehicle_id": 90,
            },
            {
                "event_id": "restore_exit_001",
                "sim_time": 11.0,
                "event_type": "exiting",
                "source": "static_obstacle_restore",
                "status": "spawned",
                "intent_observable": False,
                "vehicle_id": 91,
                "spot_index": 12,
            },
        ])
        trace = []
        for index in range(7):
            trace.append({
                "sim_time": float(index),
                "vehicle_id": 1,
                "vehicle_role": "av_entering",
                "intent_observable": True,
                "task": "CRUISE",
                "speed": 0.0,
                "is_braking": True,
                "waiting_for": 9,
                "deadlock_release_count": 1 if index == 6 else 0,
            })
        _write_jsonl(run_dir / "logs" / "vehicle_1_trace.jsonl", trace)

        paths = _discover_epoch_paths(suite, None)
        assert paths == [run_dir / "fleet_epochs.jsonl"]
        events = _critical_events(paths, suite)
        event_types = {row["event_type"] for row in events}
        assert {
            "duplicate_target_proposal",
            "feedback_repair_gain",
            "fallback_or_duplicate_target",
            "high_conflict_action_executed",
            "shield_rejection",
            "fleet_synchronized_wait",
            "high_intent_uncertainty",
        }.issubset(event_types), event_types
        assert all(row["sim_time"] == "10.000" for row in events)
        assert any(row["linked_traffic_event"].startswith("enter_001") for row in events)

        traffic = _critical_traffic_events([run_dir], suite)
        assert {row["event_type"] for row in traffic} == {
            "traffic_demand_delayed",
            "restored_obstacle_exit",
        }
        intervals = _critical_trajectory_intervals([run_dir], suite, 5.0)
        assert any(row["event_type"] == "blocked_wait_interval" and row["duration_s"] == "6.000" for row in intervals), intervals
        assert any(row["event_type"] == "deadlock_release" for row in intervals)

        summary = Path(tmp) / "summary.md"
        _write_summary(summary, [], events, traffic, intervals)
        text = summary.read_text()
        assert "关键状态分析" in text
        assert "不使用 Qwen 墙钟响应时间" in text
    print("parksim.vla critical-state analysis smoke ok")


if __name__ == "__main__":
    main()
