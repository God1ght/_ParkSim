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
    wait_before_departure = _wait_before_departure(action)
    euclidean = _distance(ego_xy, target_xy) if target_xy is not None else 0.0
    route_length = _estimate_route_length(vehicle, action, target_xy, fallback=euclidean)
    nearby_count, min_vehicle_distance, conflict_risk, conflict_vehicle_count, min_ttc = _nearby_vehicle_risk(
        vehicle, ego_xy, target_xy, wait_before_departure=wait_before_departure
    )
    if action.action_type == VLAActionType.WAIT:
        route_length = 0.0
        euclidean = 0.0
        conflict_risk = max(conflict_risk, 0.05)
    estimated_time = _estimated_time(vehicle, action, route_length, wait_before_departure)
    expected_wait = _expected_wait_seconds(action, conflict_risk, min_ttc, nearby_count)
    bundle_cost = _bundle_cost(action, route_length, estimated_time, expected_wait, conflict_risk, conflict_vehicle_count, spot_status)
    reservation_cost = _reservation_cost(action, bundle_cost, expected_wait, conflict_risk, conflict_vehicle_count)
    rolling_horizon_cost = _rolling_horizon_cost(action, bundle_cost, expected_wait, conflict_risk, conflict_vehicle_count)
    centralized_assignment_cost = _centralized_assignment_cost(action, bundle_cost, nearby_count, conflict_vehicle_count, spot_status)
    oracle_enabled = bool(getattr(vehicle, "reveal_background_intents_to_vla", False))
    oracle_risk, oracle_count, oracle_spot_competition = _oracle_intent_features(
        vehicle, ego_xy, target_xy, wait_before_departure=wait_before_departure, enabled=oracle_enabled
    )
    oracle_bundle_cost = _oracle_bundle_cost(bundle_cost, oracle_risk, oracle_count, oracle_spot_competition)
    rule_prior = _rule_prior_score(action, route_length, estimated_time, expected_wait, conflict_risk, conflict_vehicle_count, spot_status)
    assignment_id = _assignment_id(action)
    route_id = str(action.route_id or ("wait" if action.action_type == VLAActionType.WAIT else "direct"))
    return {
        "assignment_id": assignment_id,
        "route_id": route_id,
        "route_strategy": _route_strategy(action),
        "wait_before_departure_s": round(float(wait_before_departure), 3),
        "spot_distance_m": round(float(euclidean), 3),
        "route_length_m": round(float(route_length), 3),
        "estimated_time_s": round(float(estimated_time), 3),
        "expected_wait_s": round(float(expected_wait), 3),
        "nearby_vehicle_count": int(nearby_count),
        "conflict_vehicle_count": int(conflict_vehicle_count),
        "min_vehicle_distance_m": round(float(min_vehicle_distance), 3) if math.isfinite(min_vehicle_distance) else None,
        "min_ttc_s": round(float(min_ttc), 3) if math.isfinite(min_ttc) else None,
        "conflict_risk": round(float(conflict_risk), 4),
        "occupancy_status": (spot_status or {}).get("status"),
        "selectable": bool((spot_status or {}).get("selectable", action.target_spot_index is None)),
        "bundle_cost": round(float(bundle_cost), 4),
        "reservation_cost": round(float(reservation_cost), 4),
        "rolling_horizon_cost": round(float(rolling_horizon_cost), 4),
        "centralized_assignment_cost": round(float(centralized_assignment_cost), 4),
        "oracle_intent_available": bool(oracle_enabled),
        "oracle_conflict_risk": round(float(oracle_risk), 4) if oracle_enabled else None,
        "oracle_conflict_vehicle_count": int(oracle_count) if oracle_enabled else None,
        "oracle_spot_competition": round(float(oracle_spot_competition), 4) if oracle_enabled else None,
        "oracle_bundle_cost": round(float(oracle_bundle_cost), 4) if oracle_enabled else None,
        "rule_prior_score": round(float(rule_prior), 4),
        "assignment_bundle": {
            "assignment_id": assignment_id,
            "action_id": action.action_id,
            "spot_index": action.target_spot_index,
            "route_id": route_id,
            "route_strategy": _route_strategy(action),
            "wait_before_departure_s": round(float(wait_before_departure), 3),
            "path_length_m": round(float(route_length), 3),
            "eta_s": round(float(estimated_time), 3),
            "expected_wait_s": round(float(expected_wait), 3),
            "conflict_risk": round(float(conflict_risk), 4),
            "conflict_vehicle_count": int(conflict_vehicle_count),
            "bundle_cost": round(float(bundle_cost), 4),
            "reservation_cost": round(float(reservation_cost), 4),
            "rolling_horizon_cost": round(float(rolling_horizon_cost), 4),
            "centralized_assignment_cost": round(float(centralized_assignment_cost), 4),
            "oracle_bundle_cost": round(float(oracle_bundle_cost), 4) if oracle_enabled else None,
            "oracle_intent_available": bool(oracle_enabled),
            "observable_only": not bool(oracle_enabled),
        },
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


def _estimated_time(vehicle: Any, action: VLACandidateAction, route_length: float, wait_before_departure: float) -> float:
    if action.action_type == VLAActionType.WAIT:
        return float(action.duration or 0.0)
    v = float(getattr(getattr(vehicle, "vehicle_config", None), "v_cruise", 5.0) or 5.0)
    return wait_before_departure + route_length / max(v, 0.1)


def _wait_before_departure(action: VLACandidateAction) -> float:
    if action.action_type == VLAActionType.WAIT:
        return 0.0
    return max(0.0, float(action.duration or 0.0))


def _nearby_vehicle_risk(
    vehicle: Any,
    ego_xy: np.ndarray,
    target_xy: Optional[np.ndarray],
    wait_before_departure: float = 0.0,
) -> Tuple[int, float, float, int, float]:
    states = getattr(vehicle, "other_state", {}) or {}
    if not states:
        return 0, float("inf"), 0.0, 0, float("inf")
    min_dist = float("inf")
    min_ttc = float("inf")
    risk = 0.0
    count = 0
    conflict_count = 0
    for state in states.values():
        other_now = np.asarray([state.x.x, state.x.y], dtype=float)
        other = _predict_xy(state, wait_before_departure)
        d_ego = float(np.linalg.norm(other_now - ego_xy))
        d_after_wait = float(np.linalg.norm(other - ego_xy))
        min_dist = min(min_dist, d_ego, d_after_wait)
        if min(d_ego, d_after_wait) < 20.0:
            count += 1
            risk += max(0.0, (20.0 - min(d_ego, d_after_wait)) / 20.0) * 0.2
        if target_xy is not None:
            vehicle_conflicts = 0
            d_path = _point_to_segment_distance(other, ego_xy, target_xy)
            if d_path < 8.0:
                vehicle_conflicts = 1
                risk += max(0.0, (8.0 - d_path) / 8.0) * 0.85
            ttc = _time_to_path_conflict(state, ego_xy, target_xy)
            if ttc is not None:
                min_ttc = min(min_ttc, ttc)
                if ttc <= 4.0 + wait_before_departure:
                    vehicle_conflicts = 1
                    risk += max(0.0, (4.0 + wait_before_departure - ttc) / max(4.0 + wait_before_departure, 0.1)) * 0.5
            conflict_count += vehicle_conflicts
    return count, min_dist, min(1.0, risk), conflict_count, min_ttc


def _predict_xy(state: Any, dt: float) -> np.ndarray:
    xy = np.asarray([state.x.x, state.x.y], dtype=float)
    speed = float(getattr(getattr(state, "v", None), "v", 0.0) or 0.0)
    yaw = float(getattr(getattr(state, "e", None), "psi", 0.0) or 0.0)
    return xy + np.asarray([math.cos(yaw), math.sin(yaw)], dtype=float) * speed * max(0.0, float(dt))


def _time_to_path_conflict(state: Any, ego_xy: np.ndarray, target_xy: np.ndarray, horizon: float = 6.0) -> Optional[float]:
    speed = float(getattr(getattr(state, "v", None), "v", 0.0) or 0.0)
    if abs(speed) < 1e-3:
        return None
    yaw = float(getattr(getattr(state, "e", None), "psi", 0.0) or 0.0)
    velocity = np.asarray([math.cos(yaw), math.sin(yaw)], dtype=float) * speed
    start = np.asarray([state.x.x, state.x.y], dtype=float)
    samples = 12
    best_t = None
    for idx in range(samples + 1):
        t = horizon * idx / samples
        point = start + velocity * t
        if _point_to_segment_distance(point, ego_xy, target_xy) < 5.0:
            best_t = t
            break
    return best_t


def _point_to_segment_distance(point: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom <= 1e-9:
        return float(np.linalg.norm(point - a))
    t = max(0.0, min(1.0, float(np.dot(point - a, ab) / denom)))
    projection = a + t * ab
    return float(np.linalg.norm(point - projection))


def _expected_wait_seconds(action: VLACandidateAction, conflict_risk: float, min_ttc: float, nearby_count: int) -> float:
    if action.action_type == VLAActionType.WAIT:
        return float(action.duration or 0.0)
    wait = max(0.0, float(action.duration or 0.0))
    wait += 8.0 * max(0.0, float(conflict_risk))
    if math.isfinite(min_ttc) and min_ttc < 4.0:
        wait += max(0.0, 4.0 - min_ttc)
    wait += min(3.0, 0.25 * float(nearby_count))
    return wait


def _reservation_cost(
    action: VLACandidateAction,
    bundle_cost: float,
    expected_wait: float,
    conflict_risk: float,
    conflict_vehicle_count: int,
) -> float:
    if action.action_type == VLAActionType.WAIT:
        return bundle_cost + 0.5 * expected_wait
    return bundle_cost + 20.0 * conflict_risk + 4.0 * conflict_vehicle_count + 0.8 * expected_wait


def _rolling_horizon_cost(
    action: VLACandidateAction,
    bundle_cost: float,
    expected_wait: float,
    conflict_risk: float,
    conflict_vehicle_count: int,
) -> float:
    cost = bundle_cost + 0.5 * expected_wait + 10.0 * conflict_risk + 2.0 * conflict_vehicle_count
    if _route_strategy(action) == "yield_then_go" and conflict_risk > 0.25:
        cost -= 4.0
    return cost


def _centralized_assignment_cost(
    action: VLACandidateAction,
    bundle_cost: float,
    nearby_count: int,
    conflict_vehicle_count: int,
    spot_status: Optional[Dict[str, Any]],
) -> float:
    cost = bundle_cost + 0.8 * nearby_count + 8.0 * conflict_vehicle_count
    if spot_status and spot_status.get("status") == "available":
        cost -= 1.0
    if action.action_type == VLAActionType.WAIT:
        cost += 12.0
    return cost


def _oracle_bundle_cost(bundle_cost: float, oracle_risk: float, oracle_count: int, oracle_spot_competition: float) -> float:
    return bundle_cost + 55.0 * oracle_risk + 6.0 * oracle_count + 30.0 * oracle_spot_competition


def _oracle_intent_features(
    vehicle: Any,
    ego_xy: np.ndarray,
    target_xy: Optional[np.ndarray],
    wait_before_departure: float = 0.0,
    enabled: bool = False,
) -> Tuple[float, int, float]:
    if not enabled or target_xy is None:
        return 0.0, 0, 0.0
    other_states = getattr(vehicle, "other_state", {}) or {}
    other_tasks = getattr(vehicle, "other_task", {}) or {}
    other_progress = getattr(vehicle, "other_parking_progress", {}) or {}
    other_refs = getattr(vehicle, "other_ref_pose", {}) or {}
    other_done = getattr(vehicle, "other_is_all_done", {}) or {}
    risk = 0.0
    count = 0
    spot_competition = 0.0
    for vehicle_id, state in other_states.items():
        if other_done.get(vehicle_id):
            continue
        task = str(other_tasks.get(vehicle_id) or "").upper()
        progress = str(other_progress.get(vehicle_id) or "").upper()
        ref_points = _reference_points(other_refs.get(vehicle_id))
        vehicle_conflict = False
        if ref_points:
            d_ref = _polyline_to_segment_distance(ref_points, ego_xy, target_xy)
            if d_ref < 7.0:
                vehicle_conflict = True
                risk += max(0.0, (7.0 - d_ref) / 7.0) * 0.75
            final_ref = ref_points[-1]
            d_spot = float(np.linalg.norm(final_ref - target_xy))
            if task in ("CRUISE", "PARK") and d_spot < 8.0:
                spot_competition += max(0.0, (8.0 - d_spot) / 8.0)
        if task == "UNPARK" or progress == "UNPARKING":
            predicted = _predict_xy(state, wait_before_departure)
            d_path = _point_to_segment_distance(predicted, ego_xy, target_xy)
            if d_path < 9.0:
                vehicle_conflict = True
                risk += max(0.0, (9.0 - d_path) / 9.0) * 0.65
        if vehicle_conflict:
            count += 1
    return min(1.0, risk), count, min(1.0, spot_competition)


def _reference_points(ref_pose: Any, limit: int = 16) -> List[np.ndarray]:
    if ref_pose is None:
        return []
    try:
        xs = list(getattr(ref_pose, "x", []) or [])
        ys = list(getattr(ref_pose, "y", []) or [])
    except Exception:
        return []
    points = []
    for x, y in list(zip(xs, ys))[:limit]:
        points.append(np.asarray([float(x), float(y)], dtype=float))
    return points


def _polyline_to_segment_distance(points: Iterable[np.ndarray], a: np.ndarray, b: np.ndarray) -> float:
    best = float("inf")
    for point in points:
        best = min(best, _point_to_segment_distance(np.asarray(point, dtype=float), a, b))
    return best


def _bundle_cost(
    action: VLACandidateAction,
    route_length: float,
    estimated_time: float,
    expected_wait: float,
    conflict_risk: float,
    conflict_vehicle_count: int,
    spot_status: Optional[Dict[str, Any]],
) -> float:
    if action.action_type == VLAActionType.WAIT:
        return 40.0 + 2.0 * float(action.duration or 0.0) + 20.0 * conflict_risk
    cost = route_length + 0.2 * estimated_time + 2.5 * expected_wait + 45.0 * conflict_risk + 3.0 * conflict_vehicle_count
    if action.action_type == VLAActionType.PARK:
        cost -= 8.0
    if action.action_type == VLAActionType.REROUTE:
        cost += 4.0
    if spot_status and not spot_status.get("selectable", True):
        cost += 1e4
    return cost


def _rule_prior_score(
    action: VLACandidateAction,
    route_length: float,
    estimated_time: float,
    expected_wait: float,
    conflict_risk: float,
    conflict_vehicle_count: int,
    spot_status: Optional[Dict[str, Any]],
) -> float:
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
    score -= 0.2 * expected_wait
    score -= 2.0 * conflict_risk
    score -= 0.4 * conflict_vehicle_count
    return score


def _assignment_id(action: VLACandidateAction) -> str:
    if action.target_spot_index is not None:
        return "spot_%s_route_%s" % (action.target_spot_index, action.route_id or "direct")
    return str(action.action_id)


def _route_strategy(action: VLACandidateAction) -> str:
    if action.action_type == VLAActionType.WAIT:
        return "wait_only"
    if action.duration is not None and float(action.duration) > 0.0:
        return "yield_then_go"
    if action.action_type == VLAActionType.REROUTE:
        return "reroute_current_target"
    return "direct"
