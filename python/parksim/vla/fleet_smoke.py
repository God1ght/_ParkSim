from parksim.vla.fleet_client import QwenFleetPolicyClient, QwenFleetResponseError
from parksim.vla.fleet_critic import FleetDecisionCritic
from parksim.vla.fleet_coordinator import FleetEpochCoordinator
from parksim.vla.fleet_protocol import build_fleet_decision_packet
from parksim.vla.fleet_schema import VLAFleetContext, VLAFleetDecision, VLAFleetResponse
from parksim.vla.fleet_shield import VLAFleetSafetyShield
from parksim.vla.schema import VLAActionType, VLACandidateAction, VLAContext


def _context(vehicle_id, spot_costs):
    actions = [VLACandidateAction(action_id="wait_2s_v%d" % vehicle_id, action_type=VLAActionType.WAIT, duration=2.0, features={"bundle_cost": 99.0})]
    for spot, cost in spot_costs:
        actions.append(VLACandidateAction(
            action_id="v%d_cruise_to_spot_%d" % (vehicle_id, spot),
            action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
            target_spot_index=spot,
            features={
                "bundle_cost": float(cost),
                "occupancy_status": "available",
                "selectable": True,
                "conflict_risk": 0.0,
                "conflict_vehicle_count": 0,
            },
        ))
    return VLAContext(
        instruction="choose fleet action",
        state={"ego": {"vehicle_id": vehicle_id, "task": None}},
        valid_actions=actions,
    )


def main():
    left = _context(1, [(3, 1.0), (4, 3.0)])
    right = _context(2, [(3, 2.0), (5, 4.0)])
    fleet = VLAFleetContext(
        instruction="coordinate two automated vehicles",
        state={"cloud_policy_role": "fleet_level_qwen_vla_server"},
        vehicle_contexts=[left, right],
    )
    packet = build_fleet_decision_packet(fleet)
    assert packet["protocol_version"].endswith("Fleet-Decision-v2")
    assert packet["automated_vehicle_ids"] == [1, 2]
    assert len(packet["automated_vehicles"]) == 2

    fallback = QwenFleetPolicyClient(endpoint="").decide_fleet(fleet)
    assert fallback.decision_for(1).target_spot_index == 3
    assert fallback.decision_for(2).target_spot_index == 5, fallback.to_dict()

    strict_client = QwenFleetPolicyClient(endpoint="", strict_response=True)
    try:
        strict_client.decide_fleet(fleet)
    except QwenFleetResponseError as exc:
        assert "not configured" in str(exc)
    else:
        raise AssertionError("strict fleet client accepted a missing Qwen response")
    try:
        strict_client._parse_response("not-json", fleet)
    except QwenFleetResponseError as exc:
        assert "fleet_decisions" in str(exc)
    else:
        raise AssertionError("strict fleet client accepted a malformed Qwen response")
    try:
        strict_client._parse_response(
            '{"fleet_decisions":[{"vehicle_id":1,"action_id":"wait_2s_v1"}]}', fleet
        )
    except QwenFleetResponseError as exc:
        assert "coverage mismatch" in str(exc)
    else:
        raise AssertionError("strict fleet client accepted an incomplete fleet response")
    parsed = strict_client._parse_response(
        '{"fleet_decisions":['
        '{"vehicle_id":1,"action_id":"wait_2s_v1","confidence":0.0},'
        '{"vehicle_id":2,"action_id":"wait_2s_v2","confidence":0.0}'
        ']}', fleet
    )
    assert all(item.confidence == 0.0 for item in parsed.decisions)

    duplicate = VLAFleetResponse(decisions=[
        VLAFleetDecision(vehicle_id=1, action_id="v1_cruise_to_spot_3", target_spot_index=3),
        VLAFleetDecision(vehicle_id=2, action_id="v2_cruise_to_spot_3", target_spot_index=3),
    ])
    results = VLAFleetSafetyShield().validate(duplicate, fleet)
    assert results[1][0] is True, results[1]
    assert results[2][0] is False, results[2]
    assert results[2][1].target_spot_index == 5, results[2]

    critic = FleetDecisionCritic()
    critique = critic.evaluate(duplicate, fleet)
    assert critique["needs_repair"] is True, critique
    assert critique["duplicate_target_count"] == 1, critique
    optimized = critic.optimize(fleet)
    optimized_critique = critic.evaluate(optimized, fleet)
    assert optimized_critique["hard_violation_count"] == 0, optimized_critique

    crossing_left = _context(10, [(6, 1.0)])
    crossing_right = _context(11, [(7, 1.0)])
    crossing_left.valid_actions[1].features.update({
        "route_polyline_xy": [[0.0, 0.0], [10.0, 0.0]],
        "route_start_delay_s": 0.0,
        "route_end_time_s": 10.0,
    })
    crossing_right.valid_actions[1].features.update({
        "route_polyline_xy": [[5.0, -5.0], [5.0, 5.0]],
        "route_start_delay_s": 0.0,
        "route_end_time_s": 10.0,
    })
    crossing_fleet = VLAFleetContext(
        instruction="coordinate crossing routes",
        state={},
        vehicle_contexts=[crossing_left, crossing_right],
    )
    crossing_response = VLAFleetResponse(decisions=[
        VLAFleetDecision(vehicle_id=10, action_id="v10_cruise_to_spot_6", target_spot_index=6, priority=0),
        VLAFleetDecision(vehicle_id=11, action_id="v11_cruise_to_spot_7", target_spot_index=7, priority=1),
    ])
    crossing_critique = critic.evaluate(crossing_response, crossing_fleet)
    assert crossing_critique["route_conflict_count"] == 1, crossing_critique
    crossing_shield = VLAFleetSafetyShield().validate(crossing_response, crossing_fleet)
    assert crossing_shield[10][0] is True
    assert crossing_shield[11][0] is False
    assert crossing_shield[11][1].action_type == VLAActionType.WAIT

    class ScriptedRepairClient:
        def decide_fleet(self, context):
            return duplicate

        def repair_fleet(self, context, initial_response, critique=None, mode="external_feedback"):
            assert mode == "external_feedback"
            assert critique and critique["duplicate_target_count"] == 1
            return VLAFleetResponse(decisions=[
                VLAFleetDecision(vehicle_id=1, action_id="v1_cruise_to_spot_3", target_spot_index=3),
                VLAFleetDecision(vehicle_id=2, action_id="v2_cruise_to_spot_5", target_spot_index=5),
            ])

    human_observation = {
        "vehicle_id": 90,
        "declared_operation": "entering",
        "state": {"x": 1.0, "y": 1.0, "yaw": 0.0, "speed": 1.0},
    }
    left.state["nearby_vehicles"] = [dict(human_observation), {"vehicle_id": 2, "state": {}}]
    right.state["nearby_vehicles"] = [dict(human_observation), {"vehicle_id": 1, "state": {}}]
    coordinator = FleetEpochCoordinator(decision_period=3.0)
    coordinator.register(1)
    coordinator.register(2)
    assert coordinator.should_start(0.0)
    epoch = coordinator.start(sim_time=3.0, wall_time=0.0)
    assert epoch is not None
    assert epoch.trigger_type == "event"
    assert epoch.trigger_reasons == {1: "vehicle_registered", 2: "vehicle_registered"}
    assert coordinator.accept_context({"epoch_id": epoch.epoch_id, "vehicle_id": 1, "context": left.to_dict()})
    assert coordinator.accept_context({"epoch_id": epoch.epoch_id, "vehicle_id": 2, "context": right.to_dict()})
    coordinated = coordinator.finalize(ScriptedRepairClient(), policy_mode="external_feedback")
    assert coordinated["repair_attempted"] is True
    assert coordinated["repair_success"] is True
    humans = coordinated["fleet_context"]["state"]["observable_human_vehicle_states"]
    assert [row["vehicle_id"] for row in humans] == [90], humans
    assert coordinated["fleet_context"]["state"]["human_information_boundary"]["target_spot_observable"] is False
    assert not coordinator.should_start(4.0)
    assert coordinator.request_epoch(1, "idle_task_boundary")
    assert coordinator.should_start(4.0)
    triggered = coordinator.start(sim_time=4.0, wall_time=1.0)
    assert triggered is not None
    assert triggered.trigger_type == "event"
    assert triggered.trigger_reasons == {1: "idle_task_boundary"}

    from parksim.vla.intent_belief import HumanIntentBeliefTracker
    tracker = HumanIntentBeliefTracker()
    first = tracker.update(0.0, [{
        "vehicle_id": 90,
        "declared_operation": "entering",
        "state": {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 2.0},
    }])
    second = tracker.update(1.0, [{
        "vehicle_id": 90,
        "declared_operation": "entering",
        "state": {"x": 2.0, "y": 0.0, "yaw": 0.1, "speed": 2.0},
    }])
    assert first[0]["target_spot_observable"] is False
    assert second[0]["operation_class"] == "entering"
    assert abs(sum(item["probability"] for item in second[0]["motion_hypotheses"]) - 1.0) < 1e-3
    print("parksim.vla fleet smoke ok")


if __name__ == "__main__":
    main()
