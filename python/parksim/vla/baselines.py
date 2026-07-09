from typing import List, Optional

from parksim.vla.action_space import choose_default_action
from parksim.vla.schema import VLAActionType, VLACandidateAction, VLADecision

BASELINE_AGENT_TYPES = {"greedy_nearest", "greedy_shortest_path", "risk_aware_rule"}


def select_baseline_action(actions: List[VLACandidateAction], strategy: str) -> Optional[VLACandidateAction]:
    strategy = (strategy or "risk_aware_rule").lower()
    progress_actions = [action for action in actions if action.action_type in (VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.REROUTE)]
    if strategy == "greedy_nearest":
        return _min_feature(progress_actions, "spot_distance_m") or choose_default_action(actions)
    if strategy == "greedy_shortest_path":
        return _min_feature(progress_actions, "route_length_m") or choose_default_action(actions)
    if strategy == "risk_aware_rule":
        ranked = progress_actions or actions
        return max(ranked, key=lambda action: float((action.features or {}).get("rule_prior_score", -1e9)), default=None) or choose_default_action(actions)
    return choose_default_action(actions)


def make_baseline_decision(actions: List[VLACandidateAction], strategy: str) -> VLADecision:
    action = select_baseline_action(actions, strategy)
    if action is None:
        return VLADecision(action_id="", reason="no valid baseline action", confidence=0.0, used_fallback=True)
    return VLADecision(
        action_id=action.action_id,
        reason="%s selected %s" % ((strategy or "baseline"), action.action_id),
        confidence=1.0,
        used_fallback=False,
    )


def _min_feature(actions: List[VLACandidateAction], key: str) -> Optional[VLACandidateAction]:
    if not actions:
        return None
    return min(actions, key=lambda action: float((action.features or {}).get(key, 1e9)))
