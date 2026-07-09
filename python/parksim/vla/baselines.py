from typing import List, Optional

from parksim.vla.action_space import choose_default_action
from parksim.vla.schema import VLAActionType, VLACandidateAction, VLADecision


BASELINE_STRATEGIES = {
    "greedy_nearest",
    "greedy_shortest_path",
    "risk_aware_rule",
    "bundle_risk_aware",
    "conflict_aware_bundle",
    "min_bundle_cost",
}


def select_baseline_action(actions: List[VLACandidateAction], strategy: str) -> Optional[VLACandidateAction]:
    strategy = (strategy or "risk_aware_rule").lower()
    progress_actions = [
        action
        for action in actions
        if action.action_type in (
            VLAActionType.SELECT_SPOT_AND_CRUISE,
            VLAActionType.PARK,
            VLAActionType.REROUTE,
            VLAActionType.CRUISE_TO_EXIT,
        )
    ]
    spot_progress_actions = [
        action
        for action in progress_actions
        if action.action_type in (VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.REROUTE)
    ]
    direct_spot_actions = [
        action
        for action in spot_progress_actions
        if _route_strategy(action) in ("direct", "reroute_current_target")
    ]

    if strategy == "greedy_nearest":
        return _min_feature(direct_spot_actions or spot_progress_actions, "spot_distance_m") or choose_default_action(actions)
    if strategy == "greedy_shortest_path":
        return _min_feature(direct_spot_actions or spot_progress_actions, "route_length_m") or choose_default_action(actions)
    if strategy in ("risk_aware_rule", "bundle_risk_aware", "min_bundle_cost"):
        ranked = spot_progress_actions or progress_actions or actions
        return _min_bundle_cost(ranked) or choose_default_action(actions)
    if strategy == "conflict_aware_bundle":
        ranked = spot_progress_actions or progress_actions or actions
        return _min_conflict_bundle(ranked) or choose_default_action(actions)
    return choose_default_action(actions)


def make_baseline_decision(actions: List[VLACandidateAction], strategy: str) -> VLADecision:
    action = select_baseline_action(actions, strategy)
    if action is None:
        return VLADecision(action_id="", reason="no valid baseline action", confidence=0.0, used_fallback=True, reason_code="FALLBACK_OR_RECOVERY")
    features = action.features or {}
    bundle = features.get("assignment_bundle") or {}
    reason = (
        "%s selected %s as assignment-route bundle %s with cost %.3f, conflict_risk %.3f, expected_wait %.3fs"
        % (
            (strategy or "baseline"),
            action.action_id,
            bundle.get("assignment_id", features.get("assignment_id", action.action_id)),
            float(features.get("bundle_cost", 0.0) or 0.0),
            float(features.get("conflict_risk", 0.0) or 0.0),
            float(features.get("expected_wait_s", 0.0) or 0.0),
        )
    )
    return VLADecision(
        action_id=action.action_id,
        target_spot_index=action.target_spot_index,
        reason=reason,
        confidence=1.0,
        reason_code=_reason_code_for_action(action),
    )


def _min_feature(actions: List[VLACandidateAction], key: str) -> Optional[VLACandidateAction]:
    return min(actions, key=lambda action: float((action.features or {}).get(key, 1e9)), default=None)


def _min_bundle_cost(actions: List[VLACandidateAction]) -> Optional[VLACandidateAction]:
    return min(actions, key=_bundle_cost_key, default=None)


def _min_conflict_bundle(actions: List[VLACandidateAction]) -> Optional[VLACandidateAction]:
    return min(actions, key=_conflict_key, default=None)


def _bundle_cost_key(action: VLACandidateAction) -> float:
    features = action.features or {}
    return float(features.get("bundle_cost", 1e9))


def _conflict_key(action: VLACandidateAction) -> tuple:
    features = action.features or {}
    return (
        float(features.get("conflict_risk", 1e9)),
        int(features.get("conflict_vehicle_count", 10**6)),
        float(features.get("expected_wait_s", 1e9)),
        float(features.get("bundle_cost", 1e9)),
        float(features.get("route_length_m", 1e9)),
    )


def _route_strategy(action: VLACandidateAction) -> str:
    return str((action.features or {}).get("route_strategy") or "direct")


def _reason_code_for_action(action: VLACandidateAction) -> str:
    if action.action_type == VLAActionType.SELECT_SPOT_AND_CRUISE:
        if float((action.features or {}).get("wait_before_departure_s", 0.0) or 0.0) > 0.0:
            return "YIELD_TRAFFIC"
        return "PARK_AVAILABLE"
    if action.action_type == VLAActionType.PARK:
        return "PARK_READY"
    if action.action_type == VLAActionType.REROUTE:
        return "REROUTE_CONFLICT"
    if action.action_type == VLAActionType.CRUISE_TO_EXIT:
        return "EXIT_READY"
    if action.action_type == VLAActionType.WAIT:
        return "YIELD_TRAFFIC"
    return "FALLBACK_OR_RECOVERY"
