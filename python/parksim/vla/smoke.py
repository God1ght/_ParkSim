import numpy as np

from parksim.pytypes import VehicleState
from parksim.vla.action_space import build_candidate_actions, choose_default_action
from parksim.vla.agent import QwenVLAVehicle
from parksim.vla.baselines import make_baseline_decision
from parksim.vla.decision_protocol import build_decision_packet
from parksim.vla.fleet_coordinator import FleetEpochCoordinator
from parksim.vla.fleet_shield import _progress_guard_action
from parksim.vla.schema import (
    VLA_DECISION_PROTOCOL_VERSION,
    VLAActionType,
    VLACandidateAction,
    VLADecision,
    VLAContext,
)
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


def _fleet_context_requires_known_occupancy():
    vehicle = DummyVehicle()
    vehicle.fleet_coordinator_enabled = True
    vehicle.max_candidate_spots = 2
    vehicle.decision_period = 3.0
    vehicle.entrance_coords = np.asarray([14.38, 76.21], dtype=float)
    vehicle._external_fleet_epoch = None
    vehicle._external_fleet_context = None
    vehicle._external_fleet_actions = []
    vehicle._last_fleet_context = None
    vehicle._last_fleet_response = None
    vehicle.is_all_done = lambda: False
    vehicle._has_active_low_level_maneuver = lambda: False
    vehicle._active_maneuver_replan_due = lambda sim_time: QwenVLAVehicle._active_maneuver_replan_due(vehicle, sim_time)

    vehicle.occupancy = []
    assert QwenVLAVehicle.build_fleet_epoch_context(vehicle, epoch_id=1, sim_time=0.0) is None
    assert vehicle._fleet_defer_reason == "occupancy_not_ready"
    vehicle.occupancy = [1, 0, 0]
    assert QwenVLAVehicle.build_fleet_epoch_context(vehicle, epoch_id=2, sim_time=1.0) is not None
    vehicle._has_active_low_level_maneuver = lambda: True
    vehicle._last_vla_decision_time = float("-inf")
    assert QwenVLAVehicle.build_fleet_epoch_context(vehicle, epoch_id=3, sim_time=2.0) is None
    vehicle._last_vla_decision_time = 0.0
    assert QwenVLAVehicle.build_fleet_epoch_context(vehicle, epoch_id=4, sim_time=4.0) is not None


def _occupancy_defer_retries_promptly():
    coordinator = FleetEpochCoordinator(decision_period=30.0)
    coordinator.register(1)
    epoch = coordinator.start(sim_time=10.0, wall_time=0.0)
    assert epoch is not None
    assert coordinator.defer_context({
        "epoch_id": epoch.epoch_id,
        "vehicle_id": 1,
        "ready": False,
        "defer_reason": "occupancy_not_ready",
    })
    result = coordinator.finalize(client=None)
    assert result["deferred_reasons"] == {"1": "occupancy_not_ready"}
    assert coordinator.next_epoch_sim_time == 11.0


def main():
    vehicle = DummyVehicle()
    actions = build_candidate_actions(vehicle, max_spots=2)
    assert actions, "expected valid actions"
    selected = choose_default_action(actions)
    assert selected is not None and selected.action_id in [action.action_id for action in actions]
    state = build_vla_state(vehicle, valid_actions=actions)
    assert state["ego"]["vehicle_id"] == 1
    assert state["world_model"]["occupancy_ready"] is True
    assert "cruise_to_spot_0" not in state["valid_action_ids"], "occupied spot must not be valid"
    assert "cruise_to_exit" not in state["valid_action_ids"], "entry parking task must not expose exit action"
    assert all(row["selectable"] for row in state["candidate_spots"])
    assert state["candidate_assignment_bundles"], "expected assignment-route bundles in Qwen state"
    assert state["candidate_assignment_bundles"][0]["is_valid_action"] is True
    assert all(row["route_strategy"] == "wait_only" for row in state["candidate_assignment_bundles"])
    blocked_zero = [row for row in state["blocked_nearby_spots"] if row["spot_index"] == 0][0]
    assert blocked_zero["status"] == "occupied" and not blocked_zero["selectable"]
    context = VLAContext(instruction="choose action", state=state, valid_actions=actions)
    packet = build_decision_packet(context)
    assert packet["protocol_version"] == VLA_DECISION_PROTOCOL_VERSION
    assert packet["valid_action_ids"] == state["valid_action_ids"]
    assert packet["output_schema"]["required"] == ["action_id", "target_spot_index", "reason_code", "confidence"]
    ok, action, reason = VLASafetyShield().validate(VLADecision(action_id=selected.action_id), actions, vehicle=vehicle)
    assert ok, reason
    bad, _, reason = VLASafetyShield().validate(VLADecision(action_id=selected.action_id, target_spot_index=0), actions, vehicle=vehicle)
    assert not bad
    bad, _, reason = VLASafetyShield().validate(VLADecision(action_id="missing"), actions, vehicle=vehicle)
    assert not bad
    bad_action = [action for action in actions if action.action_id == "cruise_to_spot_0"]
    assert not bad_action, "occupied spot action should not be generated"
    direct = VLACandidateAction(
        action_id="cruise_to_spot_1",
        action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
        target_spot_index=1,
        features={"bundle_cost": 50.0},
    )
    yield_action = VLACandidateAction(
        action_id="yield_2s_then_cruise_to_spot_1",
        action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
        target_spot_index=1,
        features={"bundle_cost": 10.0, "wait_before_departure_s": 2.0},
    )
    decision = make_baseline_decision([direct, yield_action], "risk_aware_rule")
    assert decision.action_id == yield_action.action_id, decision
    risky_action = VLACandidateAction(
        action_id="risky_spot",
        action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
        target_spot_index=1,
        features={"conflict_risk": 1.0, "conflict_vehicle_count": 2, "bundle_cost": 1.0},
    )
    guarded = _progress_guard_action(risky_action, [risky_action, actions[0]], set())
    assert guarded is not None and guarded.action_type == VLAActionType.WAIT
    _fleet_context_requires_known_occupancy()
    _occupancy_defer_retries_promptly()
    print("parksim.vla smoke ok")


if __name__ == "__main__":
    main()
