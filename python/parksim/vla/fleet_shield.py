from typing import Any, Dict, List, Optional, Tuple

from parksim.vla.fleet_schema import VLAFleetContext, VLAFleetDecision, VLAFleetResponse
from parksim.vla.qwen_client import _select_lowest_cost_fallback
from parksim.vla.schema import VLACandidateAction, VLAActionType
from parksim.vla.shield import VLASafetyShield


class VLAFleetSafetyShield:
    def __init__(self) -> None:
        self.single_vehicle_shield = VLASafetyShield()

    def validate(
        self,
        response: VLAFleetResponse,
        context: VLAFleetContext,
        vehicle_lookup: Optional[Dict[int, Any]] = None,
    ) -> Dict[int, Tuple[bool, Optional[VLACandidateAction], str, VLAFleetDecision]]:
        vehicle_lookup = vehicle_lookup or {}
        context_by_vehicle = _contexts_by_vehicle(context)
        decisions_by_vehicle = {int(decision.vehicle_id): decision for decision in response.decisions}
        results: Dict[int, Tuple[bool, Optional[VLACandidateAction], str, VLAFleetDecision]] = {}
        reserved_targets: Dict[int, int] = {}
        for vehicle_id, vehicle_context in context_by_vehicle.items():
            decision = decisions_by_vehicle.get(vehicle_id)
            valid_actions = list(vehicle_context.valid_actions)
            if decision is None:
                fallback = _fallback_decision(vehicle_id, valid_actions, "missing fleet decision for vehicle")
                results[vehicle_id] = (False, _action_by_id(valid_actions, fallback.action_id), "missing fleet decision for vehicle", fallback)
                continue
            ok, action, reason = self.single_vehicle_shield.validate(
                decision.to_vla_decision(),
                valid_actions,
                vehicle=vehicle_lookup.get(vehicle_id),
            )
            if ok and action is not None and _is_spot_action(action):
                spot = abs(int(action.target_spot_index))
                if spot in reserved_targets:
                    ok = False
                    reason = "fleet target_spot_index conflicts with vehicle %d" % reserved_targets[spot]
                else:
                    reserved_targets[spot] = vehicle_id
            if ok and action is not None:
                results[vehicle_id] = (True, action, "ok", decision)
                continue
            fallback = _fallback_decision(vehicle_id, valid_actions, "fleet shield fallback: " + reason, reserved_targets=set(reserved_targets.keys()))
            fallback_action = _action_by_id(valid_actions, fallback.action_id)
            if fallback_action is not None and _is_spot_action(fallback_action):
                reserved_targets[abs(int(fallback_action.target_spot_index))] = vehicle_id
            results[vehicle_id] = (False, fallback_action, reason, fallback)
        return results


def _contexts_by_vehicle(context: VLAFleetContext) -> Dict[int, Any]:
    output: Dict[int, Any] = {}
    for vehicle_context in context.vehicle_contexts:
        state = vehicle_context.state if isinstance(vehicle_context.state, dict) else {}
        ego = state.get("ego", {}) if isinstance(state, dict) else {}
        vehicle_id = int(ego.get("vehicle_id", -1))
        output[vehicle_id] = vehicle_context
    return output


def _fallback_decision(vehicle_id: int, actions: List[VLACandidateAction], reason: str, reserved_targets: Optional[set] = None) -> VLAFleetDecision:
    reserved_targets = reserved_targets or set()
    filtered = [
        action for action in actions
        if action.target_spot_index is None or abs(int(action.target_spot_index)) not in reserved_targets
    ]
    selected = _select_lowest_cost_fallback(filtered or actions)
    if selected is None:
        return VLAFleetDecision(vehicle_id=vehicle_id, action_id="", reason=reason, reason_code="FALLBACK_OR_RECOVERY", used_fallback=True)
    return VLAFleetDecision(
        vehicle_id=vehicle_id,
        action_id=selected.action_id,
        target_spot_index=selected.target_spot_index,
        reason=reason,
        reason_code="FALLBACK_OR_RECOVERY",
        confidence=0.0,
        used_fallback=True,
    )


def _action_by_id(actions: List[VLACandidateAction], action_id: str) -> Optional[VLACandidateAction]:
    for action in actions:
        if action.action_id == action_id:
            return action
    return None


def _is_spot_action(action: VLACandidateAction) -> bool:
    return action.action_type in (VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.REROUTE) and action.target_spot_index is not None
