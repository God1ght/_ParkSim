import json
import tempfile
from pathlib import Path

from parksim.vla.window_metrics import collect_window_rows, paired_window_deltas, write_outputs


def _jsonl(path, rows):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(json.dumps(row) + "\n" for row in rows))


def _trace(vehicle_id, role, agent_type, y_offset, waiting=False):
    rows = []
    for time_value in (0.0, 100.0, 200.0, 299.9, 300.0, 400.0, 500.0, 599.9):
        rows.append({
            "sim_time": time_value,
            "time": time_value,
            "vehicle_id": vehicle_id,
            "vehicle_role": role,
            "agent_type": agent_type,
            "intent_observable": True,
            "task": "CRUISE",
            "x": time_value / 100.0,
            "y": y_offset,
            "speed": 0.01 if waiting and time_value < 300.0 else 1.0,
            "waiting_for": 9 if waiting and time_value < 300.0 else 0,
            "is_braking": bool(waiting and time_value < 300.0),
        })
    return rows


def _run(benchmark, scenario, agent_type, target=False):
    run_dir = benchmark / "episodes" / scenario / agent_type
    log_dir = run_dir / "logs"
    first = _trace(1, "av_entering", agent_type, 0.0, waiting=target)
    second = _trace(2, "human_entering", "rule_based", 0.2 if target else 10.0)
    _jsonl(log_dir / "vehicle_1_trace.jsonl", first)
    _jsonl(log_dir / "vehicle_2_trace.jsonl", second)
    (log_dir / "vehicle_1_summary.json").write_text(json.dumps({
        "vehicle_id": 1,
        "vehicle_role": "av_entering",
        "agent_type": agent_type,
        "completed": True,
        "total_time": 500.0,
    }))
    (log_dir / "vehicle_2_summary.json").write_text(json.dumps({
        "vehicle_id": 2,
        "vehicle_role": "human_entering",
        "agent_type": "rule_based",
        "completed": True,
        "total_time": 550.0,
    }))
    (log_dir / "traffic_schedule.json").write_text(json.dumps({
        "long_horizon_duration": 600.0,
        "events": [],
    }))
    _jsonl(log_dir / "traffic_events.jsonl", [])
    if target:
        _jsonl(run_dir / "fleet_epochs.jsonl", [{
            "sim_time": 100.0,
            "trigger_type": "event",
            "decision_barrier_status": "applied",
            "repair_attempted": True,
            "repair_success": False,
            "fleet_decisions": [{
                "vehicle_id": 1,
                "action_id": "wait_5s",
                "used_fallback": True,
                "shield_ok": False,
            }],
        }])
    return run_dir


def main():
    with tempfile.TemporaryDirectory(prefix="parksim-window-metrics-") as tmp:
        suite = Path(tmp) / "suite"
        benchmark = suite / "benchmarks" / "mixed_medium"
        scenario = "mixed_seed0_spot7_human_enter1_human_exit0_av_enter1_av_exit0"
        baseline = "rule_based"
        target = "mllm_external_feedback"
        _run(benchmark, scenario, baseline, target=False)
        _run(benchmark, scenario, target, target=True)
        _jsonl(benchmark / "episodes.jsonl", [
            {
                "scenario_id": scenario,
                "agent_type": baseline,
                "seed": 0,
                "background_mode": "mixed",
            },
            {
                "scenario_id": scenario,
                "agent_type": target,
                "seed": 0,
                "background_mode": "mixed",
            },
        ])
        _jsonl(suite / "benchmarks.jsonl", [{
            "benchmark_dir": str(benchmark),
            "status": "completed",
        }])

        rows = collect_window_rows(suite, 300.0)
        assert len(rows) == 4, rows
        deltas = paired_window_deltas(rows, baseline, target)
        assert len(deltas) == 2, deltas
        first = next(row for row in deltas if row["window_id"] == 0)
        assert first["delta_av_waiting_time_s"] > 0.0
        assert first["delta_system_near_miss_event_count"] >= 0.0
        assert first["delta_shield_rejection_count"] == 1.0
        assert first["degradation_score"] > 0.0

        out_dir = Path(tmp) / "out"
        manifest = write_outputs(suite, out_dir, baseline, target, 300.0)
        assert manifest["row_count"] == 4
        assert manifest["paired_window_count"] == 2
        for name in (
            "window_metrics.csv",
            "window_paired_deltas.csv",
            "critical_window_rank.csv",
            "window_metrics_manifest.json",
        ):
            assert (out_dir / name).exists(), name
        assert "Qwen wall-clock" in (out_dir / "window_metrics_manifest.json").read_text()
    print("parksim.vla simulation-time window metrics smoke ok")


if __name__ == "__main__":
    main()
