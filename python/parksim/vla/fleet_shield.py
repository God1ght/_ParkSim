from typing import Any, Dict, List, Optional, Tuple

from parksim.vla.fleet_schema import VLAFleetContext, VLAFleetDecision, VLAFleetResponse
from parksim.vla.fleet_critic import FleetDecisionCritic
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
        accepted_actions: Dict[int, VLACandidateAction] = {}
        conflict_graph = FleetDecisionCritic().candidate_conflict_graph(context)
        conflict_pairs = _conflict_pairs(conflict_graph)
        ordered_vehicle_ids = sorted(
            context_by_vehicle,
            key=lambda vehicle_id: (
                decisions_by_vehicle[vehicle_id].priority if vehicle_id in decisions_by_vehicle else 10 ** 9,
                vehicle_id,
            ),
        )
        for vehicle_id in ordered_vehicle_ids:
            vehicle_context = context_by_vehicle[vehicle_id]
            decision = decisions_by_vehicle.get(vehicle_id)
            valid_actions = list(vehicle_context.valid_actions)
            if decision is None:
                blocked_action_ids = {
                    candidate.action_id for candidate in valid_actions
                    if _conflicts_with_accepted(vehicle_id, candidate, accepted_actions, conflict_pairs)
                }
                fallback = _fallback_decision(
                    vehicle_id,
                    valid_actions,
                    "missing fleet decision for vehicle",
                    reserved_targets=set(reserved_targets.keys()),
                    blocked_action_ids=blocked_action_ids,
                )
                fallback_action = _action_by_id(valid_actions, fallback.action_id)
                if fallback_action is not None:
                    accepted_actions[vehicle_id] = fallback_action
                    if _is_spot_action(fallback_action):
                        reserved_targets[abs(int(fallback_action.target_spot_index))] = vehicle_id
                results[vehicle_id] = (False, fallback_action, "missing fleet decision for vehicle", fallback)
                continue
            ok, action, reason = self.single_vehicle_shield.validate(
                decision.to_vla_decision(),
                valid_actions,
                vehicle=vehicle_lookup.get(vehicle_id),
            )
            if ok and action is not None:
                guarded_action = _progress_guard_action(action, valid_actions, set(reserved_targets.keys()), vehicle_context)
                if guarded_action is not None:
                    action = guarded_action
                    decision = _guarded_decision(vehicle_id, decision, action)
                    reason = "risk-progress guard replaced a dominated model choice with executable progress"
            if ok and action is not None and _is_spot_action(action):
                spot = abs(int(action.target_spot_index))
                if spot in reserved_targets:
                    ok = False
                    reason = "fleet target_spot_index conflicts with vehicle %d" % reserved_targets[spot]
            if ok and action is not None and _conflicts_with_accepted(vehicle_id, action, accepted_actions, conflict_pairs):
                ok = False
                reason = "fleet action has a verified pairwise route conflict with an accepted higher-priority action"
            if ok and action is not None:
                if _is_spot_action(action):
                    reserved_targets[abs(int(action.target_spot_index))] = vehicle_id
                accepted_actions[vehicle_id] = action
                results[vehicle_id] = (True, action, "ok", decision)
                continue
            blocked_action_ids = {
                candidate.action_id for candidate in valid_actions
                if _conflicts_with_accepted(vehicle_id, candidate, accepted_actions, conflict_pairs)
            }
            fallback = _fallback_decision(
                vehicle_id,
                valid_actions,
                "fleet shield fallback: " + reason,
                reserved_targets=set(reserved_targets.keys()),
                blocked_action_ids=blocked_action_ids,
            )
            fallback_action = _action_by_id(valid_actions, fallback.action_id)
            if fallback_action is not None and _is_spot_action(fallback_action):
                reserved_targets[abs(int(fallback_action.target_spot_index))] = vehicle_id
            if fallback_action is not None:
                accepted_actions[vehicle_id] = fallback_action
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


def _fallback_decision(
    vehicle_id: int,
    actions: List[VLACandidateAction],
    reason: str,
    reserved_targets: Optional[set] = None,
    blocked_action_ids: Optional[set] = None,
) -> VLAFleetDecision:
    reserved_targets = reserved_targets or set()
    blocked_action_ids = blocked_action_ids or set()
    filtered = [
        action for action in actions
        if action.action_id not in blocked_action_ids
        and (action.target_spot_index is None or abs(int(action.target_spot_index)) not in reserved_targets)
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


def _progress_guard_action(
    selected: VLACandidateAction,
    actions: List[VLACandidateAction],
    reserved_targets: set,
    vehicle_context: Optional[Any] = None,
) -> Optional[VLACandidateAction]:
    """Prefer a non-conflicting progress bundle over WAIT or a dominated risky choice."""
    selected_features = selected.features if isinstance(selected.features, dict) else {}
    selected_risk = _float_feature(selected_features, "conflict_risk", 1.0)
    selected_conflicts = _float_feature(selected_features, "conflict_vehicle_count", 1.0)
    is_wait = selected.action_type == VLAActionType.WAIT
    if _blocking_vehicle_waited_on_by_traffic(vehicle_context):
        release_action = _lowest_progress_action(actions, reserved_targets, allow_risky=True)
        if release_action is not None:
            if selected.action_id == release_action.action_id:
                return None
            return release_action
    if not is_wait and selected_risk <= 0.15 and selected_conflicts <= 0.0:
        return None
    release_action = _lowest_progress_action(actions, reserved_targets, allow_risky=False)
    if release_action is None:
        return _lowest_wait_action(actions)
    return release_action


def _lowest_progress_action(
    actions: List[VLACandidateAction],
    reserved_targets: set,
    allow_risky: bool,
) -> Optional[VLACandidateAction]:
    candidates = []
    for action in actions:
        if action.action_type not in (VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.CRUISE_TO_EXIT):
            continue
        if _is_spot_action(action) and abs(int(action.target_spot_index)) in reserved_targets:
            continue
        features = action.features if isinstance(action.features, dict) else {}
        if not allow_risky:
            if _float_feature(features, "conflict_risk", 1.0) > 0.15:
                continue
            if _float_feature(features, "conflict_vehicle_count", 1.0) > 0.0:
                continue
        candidates.append(action)
    if not candidates:
        return None
    return min(candidates, key=_progress_rank)


def _blocking_vehicle_waited_on_by_traffic(vehicle_context: Optional[Any]) -> bool:
    if vehicle_context is None:
        return False
    state = vehicle_context.state if isinstance(getattr(vehicle_context, "state", None), dict) else {}
    ego = state.get("ego", {}) if isinstance(state, dict) else {}
    try:
        vehicle_id = int(ego.get("vehicle_id", -1))
        speed = abs(float((ego.get("state") or {}).get("speed", 0.0)))
        ego_waiting_for = int(ego.get("waiting_for", 0) or 0)
    except Exception:
        return False
    if vehicle_id < 0 or speed > 0.15 or ego_waiting_for != 0:
        return False
    for nearby in state.get("nearby_vehicles", []) or []:
        try:
            if int(nearby.get("waiting_for", 0) or 0) == vehicle_id and bool(nearby.get("is_braking", False)):
                return True
        except Exception:
            continue
    return False


def _lowest_wait_action(actions: List[VLACandidateAction]) -> Optional[VLACandidateAction]:
    waits = [action for action in actions if action.action_type == VLAActionType.WAIT]
    return min(waits, key=_progress_rank, default=None)


def _guarded_decision(vehicle_id: int, previous: VLAFleetDecision, action: VLACandidateAction) -> VLAFleetDecision:
    if action.action_type == VLAActionType.WAIT:
        reason_code = "YIELD_TRAFFIC"
    elif action.action_type == VLAActionType.CRUISE_TO_EXIT:
        reason_code = "EXIT_READY"
    else:
        reason_code = "PARK_AVAILABLE"
    return VLAFleetDecision(
        vehicle_id=vehicle_id,
        action_id=action.action_id,
        target_spot_index=action.target_spot_index,
        priority=previous.priority,
        reason_code=reason_code,
        reason="risk_progress_guard: selected safe progress action or deferred until traffic clears",
        confidence=previous.confidence,
        used_fallback=previous.used_fallback,
    )


def _progress_rank(action: VLACandidateAction) -> Tuple[float, float, str]:
    features = action.features if isinstance(action.features, dict) else {}
    return (
        _float_feature(features, "bundle_cost", float("inf")),
        _float_feature(features, "estimated_time_s", float("inf")),
        action.action_id,
    )


def _float_feature(features: Dict[str, Any], key: str, default: float) -> float:
    try:
        return float(features.get(key, default))
    except Exception:
        return default


def _conflict_pairs(graph: Dict[str, Any]) -> set:
    output = set()
    for row in graph.get("edges", []):
        left = (int(row["left_vehicle_id"]), str(row["left_action_id"]))
        right = (int(row["right_vehicle_id"]), str(row["right_action_id"]))
        output.add((left, right))
        output.add((right, left))
    return output


def _conflicts_with_accepted(
    vehicle_id: int,
    action: VLACandidateAction,
    accepted: Dict[int, VLACandidateAction],
    conflict_pairs: set,
) -> bool:
    key = (int(vehicle_id), action.action_id)
    return any((key, (int(other_id), other.action_id)) in conflict_pairs for other_id, other in accepted.items())
