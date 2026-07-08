from typing import Any, Dict, Iterable, List, Optional

import numpy as np

from parksim.pytypes import VehicleState


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def encode_vehicle_state(state: Optional[VehicleState]) -> Dict[str, float]:
    if state is None:
        return {"x": 0.0, "y": 0.0, "yaw": 0.0, "speed": 0.0, "acceleration": 0.0, "steering": 0.0}
    return {
        "x": _float(state.x.x),
        "y": _float(state.x.y),
        "yaw": _float(state.e.psi),
        "speed": _float(state.v.v),
        "acceleration": _float(state.u.u_a),
        "steering": _float(state.u.u_steer),
    }


def encode_occupancy(occupancy: Optional[Iterable[Any]], limit: Optional[int] = None) -> List[int]:
    if occupancy is None:
        return []
    values = [int(bool(v)) for v in list(occupancy)]
    return values[:limit] if limit is not None else values


def nearest_spots(parking_spaces: Any, ego_state: VehicleState, occupancy: Optional[Iterable[Any]], max_spots: int) -> List[Dict[str, Any]]:
    if parking_spaces is None or ego_state is None:
        return []
    spaces = np.asarray(parking_spaces, dtype=float)
    if spaces.size == 0:
        return []
    occ = encode_occupancy(occupancy)
    ego_xy = np.asarray([ego_state.x.x, ego_state.x.y], dtype=float)
    rows = []
    for idx, xy in enumerate(spaces.reshape((-1, 2))):
        occupied = bool(occ[idx]) if idx < len(occ) else False
        dist = float(np.linalg.norm(xy - ego_xy))
        rows.append({"spot_index": int(idx), "xy": xy.tolist(), "occupied": occupied, "distance": dist})
    rows.sort(key=lambda row: row["distance"])
    return rows[:max_spots]


def encode_nearby_vehicles(vehicle: Any, max_vehicles: int = 8) -> List[Dict[str, Any]]:
    rows = []
    for vehicle_id in sorted(getattr(vehicle, "other_state", {}).keys()):
        state = vehicle.other_state[vehicle_id]
        ego = vehicle.state
        dist = float(np.linalg.norm([state.x.x - ego.x.x, state.x.y - ego.x.y]))
        rows.append({
            "vehicle_id": int(vehicle_id),
            "distance": dist,
            "state": encode_vehicle_state(state),
            "task": getattr(vehicle, "other_task", {}).get(vehicle_id),
            "parking_progress": getattr(vehicle, "other_parking_progress", {}).get(vehicle_id),
            "is_braking": bool(getattr(vehicle, "other_is_braking", {}).get(vehicle_id, False)),
            "waiting_for": int(getattr(vehicle, "other_waiting_for", {}).get(vehicle_id, 0) or 0),
        })
    rows.sort(key=lambda row: row["distance"])
    return rows[:max_vehicles]


def build_vla_state(vehicle: Any, valid_actions: Optional[List[Any]] = None, max_spots: int = 12) -> Dict[str, Any]:
    target = None
    if getattr(vehicle, "spot_index", None) is not None and getattr(vehicle, "parking_spaces", None) is not None:
        idx = abs(int(vehicle.spot_index))
        spaces = np.asarray(vehicle.parking_spaces)
        if 0 <= idx < len(spaces):
            target = {"spot_index": idx, "xy": np.asarray(spaces[idx], dtype=float).tolist()}
    return {
        "ego": {
            "vehicle_id": int(getattr(vehicle, "vehicle_id", -1)),
            "task": getattr(vehicle, "current_task", None),
            "state": encode_vehicle_state(getattr(vehicle, "state", None)),
            "target": target,
            "is_braking": bool(getattr(vehicle, "is_braking", False)),
            "waiting_for": int(getattr(vehicle, "waiting_for", 0) or 0),
        },
        "candidate_spots": nearest_spots(getattr(vehicle, "parking_spaces", None), getattr(vehicle, "state", None), getattr(vehicle, "occupancy", None), max_spots),
        "nearby_vehicles": encode_nearby_vehicles(vehicle),
        "occupancy": encode_occupancy(getattr(vehicle, "occupancy", None), limit=128),
        "valid_action_ids": [action.action_id for action in (valid_actions or [])],
    }
