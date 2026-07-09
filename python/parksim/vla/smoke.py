import numpy as np

from parksim.pytypes import VehicleState
from parksim.vla.action_space import build_candidate_actions, choose_default_action
from parksim.vla.baselines import make_baseline_decision
from parksim.vla.decision_protocol import build_decision_packet
from parksim.vla.schema import VLA_DECISION_PROTOCOL_VERSION, VLADecision, VLAContext
from parksim.vla.shield import VLASafetyShield
from parksim.vla.state_encoder import build_vla_state


class DummyVehicle:
    def __init__(self):
        self.vehicle_id = 1
        self.state = VehicleState()
        self.state.x.x = 0.0
        self.state.x.y = 0.0
        self.state.e.psi = 0.0
        self.current_task = None
        self.is_braking = False
        self.waiting_for = 0
        self.other_state = {}
        self.other_task = {}
        self.other_parking_progress = {}
        self.other_is_braking = {}
        self.other_waiting_for = {}
        self.parking_spaces = np.asarray([[5.0, 0.0], [10.0, 0.0], [20.0, 0.0]], dtype=float)
        self.occupancy = [1, 0, 0]
        self.spot_index = None
        self.vehicle_role = "controlled_ego"


def main():
    vehicle = DummyVehicle()
    actions = build_candidate_actions(vehicle, max_spots=2)
    assert actions, "expected valid actions"
    selected = choose_default_action(actions)
    assert selected is not None and selected.target_spot_index == 1, "expected nearest empty spot"
    state = build_vla_state(vehicle, valid_actions=actions)
    assert state["ego"]["vehicle_id"] == 1
    assert state["world_model"]["occupancy_ready"] is True
    assert "cruise_to_spot_0" not in state["valid_action_ids"], "occupied spot must not be valid"
    assert "cruise_to_exit" not in state["valid_action_ids"], "entry parking task must not expose exit action"
    assert all(row["selectable"] for row in state["candidate_spots"])
    assert state["candidate_assignment_bundles"], "expected assignment-route bundles in Qwen state"
    assert state["candidate_assignment_bundles"][0]["is_valid_action"] is True
    assert any(row["route_strategy"] == "yield_then_go" for row in state["candidate_assignment_bundles"]), "expected yield bundle candidates"
    blocked_zero = [row for row in state["blocked_nearby_spots"] if row["spot_index"] == 0][0]
    assert blocked_zero["status"] == "occupied" and not blocked_zero["selectable"]
    context = VLAContext(instruction="choose action", state=state, valid_actions=actions)
    packet = build_decision_packet(context)
    assert packet["protocol_version"] == VLA_DECISION_PROTOCOL_VERSION
    assert packet["valid_action_ids"] == state["valid_action_ids"]
    assert packet["output_schema"]["required"] == ["action_id", "target_spot_index", "reason_code", "confidence"]
    ok, action, reason = VLASafetyShield().validate(VLADecision(action_id=selected.action_id, target_spot_index=1), actions, vehicle=vehicle)
    assert ok, reason
    bad, _, reason = VLASafetyShield().validate(VLADecision(action_id=selected.action_id, target_spot_index=0), actions, vehicle=vehicle)
    assert not bad and "does not match" in reason
    bad, _, reason = VLASafetyShield().validate(VLADecision(action_id="missing"), actions, vehicle=vehicle)
    assert not bad
    bad_action = [action for action in actions if action.action_id == "cruise_to_spot_0"]
    assert not bad_action, "occupied spot action should not be generated"
    direct = [action for action in actions if action.action_id == "cruise_to_spot_1"][0]
    yield_action = [action for action in actions if action.action_id == "yield_2s_then_cruise_to_spot_1"][0]
    direct.features["bundle_cost"] = 50.0
    yield_action.features["bundle_cost"] = 10.0
    decision = make_baseline_decision([direct, yield_action], "risk_aware_rule")
    assert decision.action_id == yield_action.action_id, decision
    print("parksim.vla smoke ok")


if __name__ == "__main__":
    main()
