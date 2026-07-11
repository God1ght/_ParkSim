from typing import Any, List, Optional, Sequence

import numpy as np

from parksim.vehicle_types import VehicleTask
from parksim.vla.action_features import enrich_candidate_actions
from parksim.vla.schema import VLAActionType, VLACandidateAction
from parksim.vla.spot_status import nearest_selectable_spot_indices


def empty_spot_indices(vehicle: Any, max_spots: int = 8) -> List[int]:
    return nearest_selectable_spot_indices(vehicle, max_spots=max_spots)


def build_candidate_actions(
    vehicle: Any,
    max_spots: int = 8,
    wait_durations: Sequence[float] = (2.0, 5.0),
    exit_coords: Optional[Any] = None,
) -> List[VLACandidateAction]:
    actions: List[VLACandidateAction] = []
    wait_actions: List[VLACandidateAction] = []
    for duration in wait_durations:
        wait_actions.append(VLACandidateAction(
            action_id="wait_%ss" % int(duration),
            action_type=VLAActionType.WAIT,
            duration=float(duration),
            reason="Yield temporarily and re-evaluate traffic and parking availability.",
        ))
    for spot_index in empty_spot_indices(vehicle, max_spots=max_spots):
        if not _is_reachable_spot(vehicle, spot_index):
            continue
        actions.append(VLACandidateAction(
            action_id="cruise_to_spot_%d" % spot_index,
            action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
            target_spot_index=int(spot_index),
            route_id="direct",
            reason="Cruise directly to a verified available candidate parking spot.",
        ))
        for duration in wait_durations:
            actions.append(VLACandidateAction(
                action_id="yield_%ss_then_cruise_to_spot_%d" % (int(duration), spot_index),
                action_type=VLAActionType.SELECT_SPOT_AND_CRUISE,
                target_spot_index=int(spot_index),
                duration=float(duration),
                route_id="yield_%ss" % int(duration),
                reason="Yield before cruising to this spot when short-term traffic conflicts are likely.",
            ))
    if getattr(vehicle, "spot_index", None) is not None:
        spot_index = abs(int(vehicle.spot_index))
        if _safe_reached_target(vehicle):
            actions.append(VLACandidateAction(
                action_id="park_spot_%d" % spot_index,
                action_type=VLAActionType.PARK,
                target_spot_index=spot_index,
                reason="Start parking after reaching the pre-parking pose.",
            ))
        actions.append(VLACandidateAction(
            action_id="reroute_spot_%d" % spot_index,
            action_type=VLAActionType.REROUTE,
            target_spot_index=spot_index,
            reason="Recompute the route to the current target spot.",
        ))
        if not _is_reachable_spot(vehicle, spot_index):
            actions.pop()
    if _has_committed_parking_target(vehicle):
        committed_spot = abs(int(vehicle.spot_index))
        actions = [
            action for action in actions
            if action.target_spot_index is None or int(action.target_spot_index) == committed_spot
        ]
    if exit_coords is not None and _allow_cruise_to_exit(vehicle):
        actions = [
            action for action in actions
            if action.action_type not in (VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.REROUTE)
        ]
        actions.append(VLACandidateAction(
            action_id="cruise_to_exit",
            action_type=VLAActionType.CRUISE_TO_EXIT,
            target_coords=np.asarray(exit_coords, dtype=float).reshape(-1).tolist(),
            reason="Cruise to the parking-lot exit coordinates for an exiting vehicle task.",
        ))
    actions.extend(wait_actions)
    return enrich_candidate_actions(vehicle, actions)


def _is_reachable_spot(vehicle: Any, spot_index: int) -> bool:
    """Accept a cruise bundle only when the directed A* graph has a route."""
    try:
        graph = getattr(vehicle, "graph", None)
        spaces = np.asarray(getattr(vehicle, "parking_spaces", None), dtype=float).reshape((-1, 2))
        idx = abs(int(spot_index))
        if graph is None or idx < 0 or idx >= len(spaces):
            return False
        start_xy = np.asarray([vehicle.state.x.x, vehicle.state.x.y], dtype=float)
        start_idx = graph.search(start_xy)
        is_north = any(idx >= low and idx <= high for low, high in getattr(vehicle, "north_spot_idx_ranges", []))
        y_offset = -float(getattr(vehicle, "spot_y_offset", 0.0)) if is_north else float(getattr(vehicle, "spot_y_offset", 0.0))
        target_xy = np.asarray([spaces[idx][0], spaces[idx][1] + y_offset], dtype=float)
        goal_idx = graph.search(target_xy)
        if start_idx == goal_idx:
            return _safe_reached_target(vehicle)
        from parksim.route_planner.a_star import AStarPlanner

        solution = AStarPlanner(graph.vertices[start_idx], graph.vertices[goal_idx]).solve()
        if not bool(getattr(solution, "edges", [])):
            return False
        x_ref, y_ref, yaw_ref = vehicle.compute_ref_path(solution, spot_index=int(spot_index))
        return len(x_ref) >= 2 and len(y_ref) >= 2 and len(yaw_ref) >= 2
    except Exception:
        return False


def choose_default_action(actions: List[VLACandidateAction]) -> Optional[VLACandidateAction]:
    for preferred in (VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.REROUTE, VLAActionType.CRUISE_TO_EXIT, VLAActionType.WAIT):
        for action in actions:
            if action.action_type == preferred:
                return action
    return actions[0] if actions else None


def apply_candidate_action(vehicle: Any, action: VLACandidateAction) -> None:
    if action.action_type == VLAActionType.WAIT:
        if str(getattr(vehicle, "current_task", "") or "").upper() in ("CRUISE", "PARK", "UNPARK"):
            setattr(vehicle, "_vla_execution_recovery_reason", "")
            return
        vehicle.set_task_profile([VehicleTask(name="IDLE", duration=float(action.duration or 2.0))])
    elif action.action_type in (VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.REROUTE):
        spot_index = int(action.target_spot_index)
        tasks = []
        if action.duration is not None and float(action.duration) > 0.0:
            tasks.append(VehicleTask(name="IDLE", duration=float(action.duration)))
        tasks.extend([
            VehicleTask(name="CRUISE", v_cruise=getattr(vehicle.vehicle_config, "v_cruise", 5), target_spot_index=spot_index),
            VehicleTask(name="PARK", target_spot_index=spot_index),
        ])
        vehicle.set_task_profile(tasks)
    elif action.action_type == VLAActionType.PARK:
        spot_index = int(action.target_spot_index if action.target_spot_index is not None else abs(vehicle.spot_index))
        vehicle.set_task_profile([VehicleTask(name="PARK", target_spot_index=spot_index)])
    elif action.action_type == VLAActionType.CRUISE_TO_EXIT:
        vehicle.set_task_profile([VehicleTask(name="CRUISE", v_cruise=getattr(vehicle.vehicle_config, "v_cruise", 5), target_coords=np.asarray(action.target_coords, dtype=float))])
    else:
        raise ValueError("Unsupported VLA action type: %s" % action.action_type)
    setattr(vehicle, "_vla_execution_recovery_reason", "")
    try:
        vehicle.execute_next_task()
    except Exception as exc:
        recovery_reason = "executor_recovery:%s" % type(exc).__name__
        setattr(vehicle, "_vla_execution_recovery_reason", recovery_reason)
        vehicle.set_task_profile([VehicleTask(name="IDLE", duration=2.0)])
        vehicle.execute_next_task()


def _allow_cruise_to_exit(vehicle: Any) -> bool:
    role = str(getattr(vehicle, "vehicle_role", "") or "").lower()
    if "exiting" in role or role in ("exit", "rule_exiting"):
        return True
    current_task = str(getattr(vehicle, "current_task", "") or "").upper()
    if current_task in ("UNPARK", "CRUISE_TO_EXIT"):
        return True
    try:
        return int(getattr(vehicle, "spot_index", 0) or 0) < 0
    except Exception:
        return False


def _has_committed_parking_target(vehicle: Any) -> bool:
    try:
        spot_index = int(getattr(vehicle, "spot_index", 0) or 0)
        if spot_index <= 0:
            return False
    except Exception:
        return False
    current_task = str(getattr(vehicle, "current_task", "") or "").upper()
    if current_task in ("CRUISE", "PARK"):
        return True
    if current_task != "IDLE":
        return False
    try:
        spaces = np.asarray(getattr(vehicle, "parking_spaces", None), dtype=float).reshape((-1, 2))
        idx = abs(spot_index)
        if idx >= len(spaces):
            return False
        xy = np.asarray([vehicle.state.x.x, vehicle.state.x.y], dtype=float)
        return float(np.linalg.norm(xy - spaces[idx])) <= max(8.0, 2.0 * float(getattr(vehicle, "spot_y_offset", 0.0) or 0.0))
    except Exception:
        return False


def _safe_reached_target(vehicle: Any) -> bool:
    try:
        return bool(vehicle.reached_target())
    except Exception:
        return False
