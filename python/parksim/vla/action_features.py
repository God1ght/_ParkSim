from typing import Any, Dict, Iterable, List, Optional, Tuple

import math
import numpy as np

from parksim.vla.schema import VLAActionType, VLACandidateAction
from parksim.vla.spot_status import status_by_index


def enrich_candidate_actions(vehicle: Any, actions: List[VLACandidateAction]) -> List[VLACandidateAction]:
    for action in actions:
        action.features = build_action_features(vehicle, action)
    return actions


def build_action_features(vehicle: Any, action: VLACandidateAction) -> Dict[str, Any]:
    ego_xy = _ego_xy(vehicle)
    target_xy = _target_xy(vehicle, action)
    spot_status = None
    if action.target_spot_index is not None:
        spot_status = status_by_index(vehicle).get(abs(int(action.target_spot_index)))
    euclidean = _distance(ego_xy, target_xy) if target_xy is not None else 0.0
    route_length = _estimate_route_length(vehicle, action, target_xy, fallback=euclidean)
    nearby_count, min_vehicle_distance, conflict_risk = _nearby_vehicle_risk(vehicle, ego_xy, target_xy)
    if action.action_type == VLAActionType.WAIT:
        route_length = 0.0
        euclidean = 0.0
        conflict_risk = max(conflict_risk, 0.05)
    estimated_time = _estimated_time(vehicle, action, route_length)
    rule_prior = _rule_prior_score(action, route_length, estimated_time, conflict_risk, spot_status)
    return {
        "spot_distance_m": round(float(euclidean), 3),
        "route_length_m": round(float(route_length), 3),
        "estimated_time_s": round(float(estimated_time), 3),
        "nearby_vehicle_count": int(nearby_count),
        "min_vehicle_distance_m": round(float(min_vehicle_distance), 3) if math.isfinite(min_vehicle_distance) else None,
        "conflict_risk": round(float(conflict_risk), 4),
        "occupancy_status": (spot_status or {}).get("status"),
        "selectable": bool((spot_status or {}).get("selectable", action.target_spot_index is None)),
        "rule_prior_score": round(float(rule_prior), 4),
    }


def _ego_xy(vehicle: Any) -> np.ndarray:
    return np.asarray([vehicle.state.x.x, vehicle.state.x.y], dtype=float)


def _target_xy(vehicle: Any, action: VLACandidateAction) -> Optional[np.ndarray]:
    if action.target_coords is not None:
        return np.asarray(action.target_coords, dtype=float).reshape(-1)[:2]
    if action.target_spot_index is None or getattr(vehicle, "parking_spaces", None) is None:
        return None
    idx = abs(int(action.target_spot_index))
    spaces = np.asarray(vehicle.parking_spaces, dtype=float).reshape((-1, 2))
    if 0 <= idx < len(spaces):
        return spaces[idx]
    return None


def _distance(a: Optional[np.ndarray], b: Optional[np.ndarray]) -> float:
    if a is None or b is None:
        return 0.0
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def _estimate_route_length(vehicle: Any, action: VLACandidateAction, target_xy: Optional[np.ndarray], fallback: float) -> float:
    if target_xy is None or getattr(vehicle, "graph", None) is None:
        return fallback
    try:
        from parksim.route_planner.a_star import AStarPlanner

        start = _ego_xy(vehicle)
        start_vertex = vehicle.graph.vertices[vehicle.graph.search(start)]
        if action.target_spot_index is not None and getattr(vehicle, "parking_spaces", None) is not None:
            spot_index = abs(int(action.target_spot_index))
            is_north = any([spot_index >= r[0] and spot_index <= r[1] for r in getattr(vehicle, "north_spot_idx_ranges", [])])
            y_offset = -float(getattr(vehicle, "spot_y_offset", 0.0)) if is_north else float(getattr(vehicle, "spot_y_offset", 0.0))
            target_for_graph = np.asarray([target_xy[0], target_xy[1] + y_offset], dtype=float)
        else:
            target_for_graph = target_xy
        goal_vertex = vehicle.graph.vertices[vehicle.graph.search(target_for_graph)]
        graph_sol = AStarPlanner(start_vertex, goal_vertex).solve()
        coords: List[np.ndarray] = [start]
        coords.extend([np.asarray(v.coords, dtype=float) for v in getattr(graph_sol, "vertices", [])])
        coords.append(np.asarray(target_for_graph, dtype=float))
        coords.append(np.asarray(target_xy, dtype=float))
        return max(fallback, _polyline_length(coords))
    except Exception:
        return fallback


def _polyline_length(points: Iterable[np.ndarray]) -> float:
    total = 0.0
    prev = None
    for point in points:
        point = np.asarray(point, dtype=float).reshape(-1)[:2]
        if prev is not None:
            total += float(np.linalg.norm(point - prev))
        prev = point
    return total


def _estimated_time(vehicle: Any, action: VLACandidateAction, route_length: float) -> float:
    if action.action_type == VLAActionType.WAIT:
        return float(action.duration or 0.0)
    v = float(getattr(getattr(vehicle, "vehicle_config", None), "v_cruise", 5.0) or 5.0)
    return route_length / max(v, 0.1)


def _nearby_vehicle_risk(vehicle: Any, ego_xy: np.ndarray, target_xy: Optional[np.ndarray]) -> Tuple[int, float, float]:
    states = getattr(vehicle, "other_state", {}) or {}
    if not states:
        return 0, float("inf"), 0.0
    min_dist = float("inf")
    risk = 0.0
    count = 0
    for state in states.values():
        other = np.asarray([state.x.x, state.x.y], dtype=float)
        d_ego = float(np.linalg.norm(other - ego_xy))
        min_dist = min(min_dist, d_ego)
        if d_ego < 20.0:
            count += 1
            risk += max(0.0, (20.0 - d_ego) / 20.0) * 0.25
        if target_xy is not None:
            d_path = _point_to_segment_distance(other, ego_xy, target_xy)
            if d_path < 6.0:
                risk += max(0.0, (6.0 - d_path) / 6.0) * 0.75
    return count, min_dist, min(1.0, risk)


def _point_to_segment_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-9:
        return float(np.linalg.norm(point - a))
    t = max(0.0, min(1.0, float(np.dot(point - a, ab) / denom)))
    projection = a + t * ab
    return float(np.linalg.norm(point - projection))


def _rule_prior_score(action: VLACandidateAction, route_length: float, estimated_time: float, conflict_risk: float, spot_status: Optional[Dict[str, Any]]) -> float:
    score = 0.0
    if action.action_type == VLAActionType.SELECT_SPOT_AND_CRUISE:
        score += 2.0
    elif action.action_type == VLAActionType.PARK:
        score += 2.5
    elif action.action_type == VLAActionType.REROUTE:
        score += 0.8
    elif action.action_type == VLAActionType.CRUISE_TO_EXIT:
        score += 0.4
    elif action.action_type == VLAActionType.WAIT:
        score -= 0.6 + 0.1 * float(action.duration or 0.0)
    if spot_status and not spot_status.get("selectable", True):
        score -= 100.0
    score -= 0.025 * route_length
    score -= 0.02 * estimated_time
    score -= 2.0 * conflict_risk
    return score
