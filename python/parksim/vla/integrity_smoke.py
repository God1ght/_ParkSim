import tempfile
from pathlib import Path

from parksim.vla.benchmark import write_metrics
from parksim.vla.safety_metrics import trace_integrity_metrics
from parksim.vla.vehicle_ids import VehicleIdAllocator, replay_vehicle_ids


def _row(time_value, x, vehicle_id=7, spawn_event_id="controlled_ego"):
    return {
        "time": float(time_value),
        "wall_time": 1000.0 + float(time_value),
        "vehicle_id": vehicle_id,
        "spawn_event_id": spawn_event_id,
        "vehicle_role": "controlled_ego",
        "agent_type": "qwen_vla",
        "is_controlled_ego": True,
        "x": float(x),
        "y": 0.0,
        "speed": 1.0,
    }


def main():
    reserved = replay_vehicle_ids([0, 1, 7, 20, "bad"], "mixed")
    assert reserved == {0, 1, 7, 20}
    allocator = VehicleIdAllocator(reserved)
    assert allocator.allocate() == 21
    assert allocator.claim_reserved(7) == 7
    try:
        allocator.claim_reserved(7)
    except ValueError:
        pass
    else:
        raise AssertionError("duplicate replay ID must be rejected")

    valid = trace_integrity_metrics([(Path("vehicle_21_trace.jsonl"), [_row(0, 0, 21), _row(1, 1, 21)])])
    assert valid["trace_integrity_ok"] is True
    invalid_rows = [_row(1, 0), _row(0, 20, spawn_event_id="replay_7")]
    invalid = trace_integrity_metrics([(Path("vehicle_7_trace.jsonl"), invalid_rows)])
    assert invalid["trace_integrity_ok"] is False
    assert invalid["trace_identity_conflict_count"] == 1
    assert invalid["trace_time_regression_count"] == 1
    jump = trace_integrity_metrics([(Path("vehicle_21_trace.jsonl"), [_row(0, 0, 21), _row(1, 20, 21)])])
    assert jump["trace_integrity_ok"] is False
    assert jump["trace_kinematic_jump_count"] == 1

    with tempfile.TemporaryDirectory() as tmp:
        out_dir = Path(tmp)
        write_metrics(out_dir, [{"scenario_id": "a", "agent_type": "rule_based", "value": 1}, {"scenario_id": "a", "agent_type": "qwen_vla", "value": 2}])
        header = (out_dir / "metrics.csv").read_text().splitlines()[0].split(",")
        assert len(header) == len(set(header)), "metrics.csv header names must be unique"
    print("parksim.vla integrity smoke ok")


if __name__ == "__main__":
    main()
