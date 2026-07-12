"""Deterministic critic and non-LLM fleet baseline for cloud coordination."""
from typing import Any, Dict, List, Optional, Tuple

import hashlib
import json
import math

from parksim.vla.fleet_schema import VLAFleetContext, VLAFleetDecision, VLAFleetResponse
from parksim.vla.schema import VLACandidateAction, VLAActionType


class FleetDecisionCritic:
    """Checks executable fleet proposals using only observable structured state."""

    def __init__(self, safety_distance_m: float = 3.5, dominated_margin: float = 15.0) -> None:
        self.safety_distance_m = max(0.1, float(safety_distance_m))
        self.dominated_margin = max(0.0, float(dominated_margin))

    def candidate_conflict_graph(self, context: VLAFleetContext) -> Dict[str, Any]:
        contexts = _contexts_by_vehicle(context)
        edges: List[Dict[str, Any]] = []
        vehicle_ids = sorted(contexts)
        for left_index, left_id in enumerate(vehicle_ids):
            for right_id in vehicle_ids[left_index + 1:]:
                for left in contexts[left_id].valid_actions:
                    for right in contexts[right_id].valid_actions:
                        conflict = self._pairwise_conflict(left, right)
                        if conflict is None:
                            continue
                        edges.append({
                            "left_vehicle_id": int(left_id),
                            "left_action_id": left.action_id,
                            "right_vehicle_id": int(right_id),
                            "right_action_id": right.action_id,
                            **conflict,
                        })
        edges.sort(key=lambda row: (
            int(row["left_vehicle_id"]), str(row["left_action_id"]),
            int(row["right_vehicle_id"]), str(row["right_action_id"]),
        ))
        return {
            "schema_version": "ParkSim-LLM-Candidate-Conflict-Graph-v1",
            "safety_distance_m": self.safety_distance_m,
            "edge_count": len(edges),
            "edges": edges,
        }

    def evaluate(
        self,
        response: VLAFleetResponse,
        context: VLAFleetContext,
        conflict_graph: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        contexts = _contexts_by_vehicle(context)
        decisions = {int(item.vehicle_id): item for item in response.decisions}
        graph = conflict_graph or self.candidate_conflict_graph(context)
        violations: List[Dict[str, Any]] = []
        quality_warnings: List[Dict[str, Any]] = []
        selected: Dict[int, VLACandidateAction] = {}
        target_owners: Dict[int, int] = {}
        suggested: Dict[str, List[str]] = {}
        objective = 0.0

        for vehicle_id, vehicle_context in sorted(contexts.items()):
            actions = list(vehicle_context.valid_actions)
            decision = decisions.get(vehicle_id)
            if decision is None:
                violations.append(_violation("missing_vehicle_decision", [vehicle_id], "no fleet decision returned"))
                suggested[str(vehicle_id)] = [item.action_id for item in _rank_actions(actions)[:3]]
                continue
            action = _action_by_id(actions, decision.action_id)
            if decision.used_fallback:
                quality_warnings.append(_violation(
                    "service_or_client_fallback", [vehicle_id], decision.reason or decision.action_id))
            if action is None:
                violations.append(_violation("invalid_action_id", [vehicle_id], decision.action_id))
                suggested[str(vehicle_id)] = [item.action_id for item in _rank_actions(actions)[:3]]
                continue
            selected[vehicle_id] = action
            features = action.features if isinstance(action.features, dict) else {}
            objective += _float(features.get("bundle_cost"), 1e4)
            if decision.target_spot_index is not None and action.target_spot_index != decision.target_spot_index:
                violations.append(_violation("target_action_mismatch", [vehicle_id], decision.action_id))
            if features.get("selectable") is False or features.get("occupancy_status") in ("occupied", "unknown", "blocked"):
                violations.append(_violation("unavailable_target", [vehicle_id], decision.action_id))
            if _is_spot_action(action):
                spot = abs(int(action.target_spot_index))
                if spot in target_owners:
                    violations.append(_violation("duplicate_target_spot", [target_owners[spot], vehicle_id], str(spot)))
                else:
                    target_owners[spot] = vehicle_id
            risk = _float(features.get("conflict_risk"), 0.0)
            if risk >= 0.7:
                quality_warnings.append(_violation("high_observable_risk", [vehicle_id], "%.4f" % risk))
            ranked = _rank_actions(actions)
            if ranked:
                best = ranked[0]
                best_cost = _float((best.features or {}).get("bundle_cost"), 1e4)
                selected_cost = _float(features.get("bundle_cost"), 1e4)
                best_risk = _float((best.features or {}).get("conflict_risk"), 1.0)
                if selected_cost > best_cost + self.dominated_margin and best_risk <= risk:
                    quality_warnings.append(_violation("dominated_bundle", [vehicle_id], "%s>%s" % (action.action_id, best.action_id)))

        selected_ids = {(vehicle_id, action.action_id) for vehicle_id, action in selected.items()}
        for edge in graph.get("edges", []):
            if edge.get("conflict_type") == "duplicate_target_spot":
                continue
            left = (int(edge.get("left_vehicle_id", -1)), str(edge.get("left_action_id", "")))
            right = (int(edge.get("right_vehicle_id", -1)), str(edge.get("right_action_id", "")))
            if left in selected_ids and right in selected_ids:
                violations.append(_violation(
                    str(edge.get("conflict_type", "route_conflict")),
                    [left[0], right[0]],
                    "%s|%s" % (left[1], right[1]),
                ))

        for vehicle_id, vehicle_context in sorted(contexts.items()):
            suggested[str(vehicle_id)] = self._suggestions(
                vehicle_id, list(vehicle_context.valid_actions), selected, graph
            )

        hard_count = len(violations)
        warning_count = len(quality_warnings)
        objective += 500.0 * hard_count + 25.0 * warning_count
        audit = _observability_audit(context)
        return {
            "schema_version": "ParkSim-LLM-Fleet-Critic-v1",
            "needs_repair": bool(violations or quality_warnings),
            "hard_violation_count": hard_count,
            "quality_warning_count": warning_count,
            "route_conflict_count": sum(1 for item in violations if item["code"] == "pairwise_route_conflict"),
            "duplicate_target_count": sum(1 for item in violations if item["code"] == "duplicate_target_spot"),
            "violations": violations,
            "quality_warnings": quality_warnings,
            "suggested_action_ids": suggested,
            "objective_score": round(objective, 4),
            "candidate_conflict_edge_count": int(graph.get("edge_count", 0)),
            "observability_audit": audit,
            "critique_hash": _stable_hash({"violations": violations, "warnings": quality_warnings, "suggestions": suggested}),
        }

    def optimize(self, context: VLAFleetContext) -> VLAFleetResponse:
        """Deterministic regret-ordered fleet min-cost baseline with hard conflict exclusion."""
        contexts = _contexts_by_vehicle(context)
        graph = self.candidate_conflict_graph(context)
        conflict_pairs = _conflict_pairs(graph)
        ordered = sorted(contexts, key=lambda vehicle_id: (-_regret(contexts[vehicle_id].valid_actions), vehicle_id))
        selected: Dict[int, VLACandidateAction] = {}
        decisions: List[VLAFleetDecision] = []
        for priority, vehicle_id in enumerate(ordered):
            ranked = _rank_actions(list(contexts[vehicle_id].valid_actions))
            action = next((candidate for candidate in ranked if not _conflicts_with_selected(vehicle_id, candidate, selected, conflict_pairs)), None)
            if action is None:
                action = next((candidate for candidate in ranked if candidate.action_type == VLAActionType.WAIT), ranked[0] if ranked else None)
            decisions.append(VLAFleetDecision(
                vehicle_id=vehicle_id,
                action_id=action.action_id if action is not None else "",
                target_spot_index=action.target_spot_index if action is not None else None,
                priority=priority,
                reason_code=_reason_code(action),
                reason="deterministic fleet min-cost selection with pairwise conflict exclusion",
                confidence=1.0,
                used_fallback=False,
            ))
            if action is not None:
                selected[vehicle_id] = action
        decisions.sort(key=lambda item: item.vehicle_id)
        return VLAFleetResponse(decisions=decisions, raw_response="deterministic_fleet_min_cost")

    def _pairwise_conflict(self, left: VLACandidateAction, right: VLACandidateAction) -> Optional[Dict[str, Any]]:
        if _is_spot_action(left) and _is_spot_action(right):
            if abs(int(left.target_spot_index)) == abs(int(right.target_spot_index)):
                return {"conflict_type": "duplicate_target_spot", "min_distance_m": 0.0, "conflict_time_s": 0.0}
        if left.action_type == VLAActionType.WAIT or right.action_type == VLAActionType.WAIT:
            return None
        left_route = _route(left)
        right_route = _route(right)
        if len(left_route) < 2 or len(right_route) < 2:
            return None
        end_time = max(_route_end(left), _route_end(right))
        minimum = float("inf")
        conflict_time = None
        sample_count = max(2, int(math.ceil(end_time / 0.5)) + 1)
        for index in range(sample_count):
            t = end_time * index / max(1, sample_count - 1)
            distance = math.hypot(*(_timed_xy(left, t, left_route)[axis] - _timed_xy(right, t, right_route)[axis] for axis in (0, 1)))
            if distance < minimum:
                minimum = distance
            if conflict_time is None and distance < self.safety_distance_m:
                conflict_time = t
        if conflict_time is None:
            return None
        return {
            "conflict_type": "pairwise_route_conflict",
            "min_distance_m": round(float(minimum), 3),
            "conflict_time_s": round(float(conflict_time), 3),
        }

    def _suggestions(
        self,
        vehicle_id: int,
        actions: List[VLACandidateAction],
        selected: Dict[int, VLACandidateAction],
        graph: Dict[str, Any],
    ) -> List[str]:
        conflict_pairs = _conflict_pairs(graph)
        others = {key: value for key, value in selected.items() if key != vehicle_id}
        feasible = [action for action in _rank_actions(actions) if not _conflicts_with_selected(vehicle_id, action, others, conflict_pairs)]
        return [action.action_id for action in feasible[:3]]


def _contexts_by_vehicle(context: VLAFleetContext) -> Dict[int, Any]:
    output = {}
    for vehicle_context in context.vehicle_contexts:
        ego = (vehicle_context.state or {}).get("ego", {})
        output[int(ego.get("vehicle_id", -1))] = vehicle_context
    return output


def _action_by_id(actions: List[VLACandidateAction], action_id: str) -> Optional[VLACandidateAction]:
    return next((action for action in actions if action.action_id == action_id), None)


def _rank_actions(actions: List[VLACandidateAction]) -> List[VLACandidateAction]:
    return sorted(actions, key=lambda action: (
        _float((action.features or {}).get("bundle_cost"), 1e9),
        _float((action.features or {}).get("conflict_risk"), 1.0),
        action.action_id,
    ))


def _regret(actions: List[VLACandidateAction]) -> float:
    ranked = _rank_actions(list(actions))
    if len(ranked) < 2:
        return 1e6
    return _float((ranked[1].features or {}).get("bundle_cost"), 1e9) - _float((ranked[0].features or {}).get("bundle_cost"), 1e9)


def _is_spot_action(action: VLACandidateAction) -> bool:
    return action.target_spot_index is not None and action.action_type in (
        VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.REROUTE,
    )


def _route(action: VLACandidateAction) -> List[Tuple[float, float]]:
    rows = (action.features or {}).get("route_polyline_xy", [])
    output = []
    for row in rows if isinstance(rows, list) else []:
        try:
            output.append((float(row[0]), float(row[1])))
        except Exception:
            continue
    return output


def _route_end(action: VLACandidateAction) -> float:
    return max(0.1, _float((action.features or {}).get("route_end_time_s"), _float((action.features or {}).get("estimated_time_s"), 1.0)))


def _timed_xy(action: VLACandidateAction, t: float, route: List[Tuple[float, float]]) -> Tuple[float, float]:
    start = max(0.0, _float((action.features or {}).get("route_start_delay_s"), 0.0))
    end = max(start + 1e-6, _route_end(action))
    progress = max(0.0, min(1.0, (float(t) - start) / (end - start)))
    lengths = [0.0]
    for previous, current in zip(route, route[1:]):
        lengths.append(lengths[-1] + math.hypot(current[0] - previous[0], current[1] - previous[1]))
    total = lengths[-1]
    if total <= 1e-9:
        return route[-1]
    target = progress * total
    for index in range(1, len(route)):
        if lengths[index] < target:
            continue
        segment = max(1e-9, lengths[index] - lengths[index - 1])
        ratio = (target - lengths[index - 1]) / segment
        return (
            route[index - 1][0] + ratio * (route[index][0] - route[index - 1][0]),
            route[index - 1][1] + ratio * (route[index][1] - route[index - 1][1]),
        )
    return route[-1]


def _conflict_pairs(graph: Dict[str, Any]) -> set:
    output = set()
    for row in graph.get("edges", []):
        left = (int(row["left_vehicle_id"]), str(row["left_action_id"]))
        right = (int(row["right_vehicle_id"]), str(row["right_action_id"]))
        output.add((left, right))
        output.add((right, left))
    return output


def _conflicts_with_selected(vehicle_id: int, action: VLACandidateAction, selected: Dict[int, VLACandidateAction], pairs: set) -> bool:
    key = (int(vehicle_id), action.action_id)
    return any((key, (int(other_id), other.action_id)) in pairs for other_id, other in selected.items())


def _reason_code(action: Optional[VLACandidateAction]) -> str:
    if action is None:
        return "FALLBACK_OR_RECOVERY"
    if action.action_type == VLAActionType.WAIT:
        return "YIELD_TRAFFIC"
    if action.action_type == VLAActionType.CRUISE_TO_EXIT:
        return "EXIT_READY"
    return "PARK_AVAILABLE"


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _violation(code: str, vehicle_ids: List[int], detail: str) -> Dict[str, Any]:
    return {"code": str(code), "vehicle_ids": [int(value) for value in vehicle_ids], "detail": str(detail)}


def _stable_hash(payload: Any) -> str:
    encoded = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _observability_audit(context: VLAFleetContext) -> Dict[str, Any]:
    state = context.state if isinstance(context.state, dict) else {}
    humans = state.get("observable_human_vehicle_states", [])
    forbidden = ("target_spot", "planned_route", "future_trajectory", "task_detail", "parking_progress")
    exposed = sorted({key for row in humans if isinstance(row, dict) for key in forbidden if row.get(key) not in (None, "unknown", [], {})})
    return {
        "human_operation_class_visible": True,
        "human_target_spot_exposed": "target_spot" in exposed,
        "human_route_exposed": "planned_route" in exposed,
        "human_future_trajectory_exposed": "future_trajectory" in exposed,
        "forbidden_exposed_fields": exposed,
        "observability_audit_ok": not exposed,
    }
