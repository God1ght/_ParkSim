from parksim.vla.fleet_client import QwenFleetPolicyClient
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
            features={"bundle_cost": float(cost), "occupancy_status": "available", "selectable": True},
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
    assert packet["protocol_version"].endswith("Fleet-Decision-v1")
    assert packet["automated_vehicle_ids"] == [1, 2]
    assert len(packet["automated_vehicles"]) == 2

    fallback = QwenFleetPolicyClient(endpoint="").decide_fleet(fleet)
    assert fallback.decision_for(1).target_spot_index == 3
    assert fallback.decision_for(2).target_spot_index == 5, fallback.to_dict()

    duplicate = VLAFleetResponse(decisions=[
        VLAFleetDecision(vehicle_id=1, action_id="v1_cruise_to_spot_3", target_spot_index=3),
        VLAFleetDecision(vehicle_id=2, action_id="v2_cruise_to_spot_3", target_spot_index=3),
    ])
    results = VLAFleetSafetyShield().validate(duplicate, fleet)
    assert results[1][0] is True, results[1]
    assert results[2][0] is False, results[2]
    assert results[2][1].target_spot_index == 5, results[2]
    print("parksim.vla fleet smoke ok")


if __name__ == "__main__":
    main()
