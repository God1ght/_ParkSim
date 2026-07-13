from parksim.vla.sync_barrier import (
    barrier_audit_fields,
    decision_barrier_status,
    validate_decision_ack,
    validate_pause_ack,
)


def main() -> None:
    run_id = "sync-smoke"
    assert validate_pause_ack(
        {"run_id": run_id, "paused": True, "sim_time": 9.9}, run_id, 10.0
    ) is None
    assert validate_pause_ack(
        {"run_id": "other", "paused": True, "sim_time": 10.0}, run_id, 10.0
    ) is None
    pause_ack = validate_pause_ack(
        {"run_id": run_id, "paused": True, "sim_time": 10.0}, run_id, 10.0
    )
    assert pause_ack is not None and pause_ack["sim_time"] == 10.0

    pending = {
        "run_id": run_id,
        "epoch_id": 3,
        "sim_time": 10.0,
        "fleet_decisions": [{"vehicle_id": 7}, {"vehicle_id": 8}],
    }
    acknowledgements = {}
    for vehicle_id in (7, 8):
        accepted = validate_decision_ack({
            "run_id": run_id,
            "epoch_id": 3,
            "vehicle_id": vehicle_id,
            "decision_sim_time": 10.0,
            "vehicle_sim_time": 10.0,
            "applied": True,
        }, run_id, pending)
        assert accepted is not None
        acknowledgements[accepted[0]] = accepted[1]
    assert decision_barrier_status(pending, acknowledgements, 2.0, 1.0, 10.0) == "applied"
    audit = barrier_audit_fields(pending, acknowledgements, 2.0, 1.0, "applied")
    assert audit["decision_ack_coverage_rate"] == 1.0
    assert audit["decision_ack_failed_vehicle_ids"] == []
    assert audit["simulation_time_policy"] == "decision_then_advance"
    assert audit["wall_clock_latency_in_performance_metrics"] is False

    mismatched = validate_decision_ack({
        "run_id": run_id,
        "epoch_id": 3,
        "vehicle_id": 7,
        "decision_sim_time": 10.0,
        "vehicle_sim_time": 10.1,
        "applied": True,
    }, run_id, pending)
    assert mismatched is not None and mismatched[1]["applied"] is False
    assert decision_barrier_status(pending, {7: mismatched[1]}, 2.0, 1.0, 10.0) == "decision_apply_failed"
    assert decision_barrier_status(pending, {}, 12.0, 1.0, 10.0) == "decision_ack_timeout"
    malformed = dict(pending, fleet_decisions=[{"vehicle_id": None}, "bad"])
    assert decision_barrier_status(malformed, {}, 2.0, 1.0, 10.0) == "applied"
    print("parksim.vla synchronous decision barrier smoke ok")


if __name__ == "__main__":
    main()
