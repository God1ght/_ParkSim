from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np


def normalize_occupancy(occupancy: Optional[Iterable[Any]]) -> List[int]:
    if occupancy is None:
        return []
    return [int(bool(value)) for value in list(occupancy)]


def parking_spaces_array(vehicle: Any) -> np.ndarray:
    spaces = getattr(vehicle, "parking_spaces", None)
    if spaces is None:
        return np.zeros((0, 2), dtype=float)
    arr = np.asarray(spaces, dtype=float)
    if arr.size == 0:
        return np.zeros((0, 2), dtype=float)
    return arr.reshape((-1, 2))


def occupancy_ready(vehicle: Any) -> bool:
    spaces = parking_spaces_array(vehicle)
    if len(spaces) == 0:
        return False
    return len(normalize_occupancy(getattr(vehicle, "occupancy", None))) >= len(spaces)


def nearest_spot_index(spaces: np.ndarray, xy: Iterable[float]) -> Tuple[Optional[int], float]:
    if len(spaces) == 0:
        return None, float("inf")
    point = np.asarray(list(xy), dtype=float).reshape(-1)[:2]
    dists = np.linalg.norm(spaces - point, axis=1)
    idx = int(np.argmin(dists))
    return idx, float(dists[idx])


def build_spot_statuses(vehicle: Any, other_vehicle_spot_radius: float = 2.2) -> List[Dict[str, Any]]:
    spaces = parking_spaces_array(vehicle)
    central = normalize_occupancy(getattr(vehicle, "occupancy", None))
    ready = len(central) >= len(spaces) and len(spaces) > 0
    ego_state = getattr(vehicle, "state", None)
    ego_xy = None
    if ego_state is not None:
        ego_xy = np.asarray([ego_state.x.x, ego_state.x.y], dtype=float)

    statuses: List[Dict[str, Any]] = []
    for idx, xy in enumerate(spaces):
        reasons: List[str] = []
        central_occupied = bool(central[idx]) if idx < len(central) else False
        known = idx < len(central)
        if not known:
            reasons.append("occupancy_unknown")
        if central_occupied:
            reasons.append("central_occupancy")
        dist_to_ego = float(np.linalg.norm(xy - ego_xy)) if ego_xy is not None else float("inf")
        statuses.append({
            "spot_index": int(idx),
            "xy": xy.tolist(),
            "distance": dist_to_ego,
            "central_occupied": central_occupied,
            "occupied_by_vehicle_ids": [],
            "reserved_by_vehicle_ids": [],
            "occupancy_known": known,
            "occupancy_ready": ready,
            "selectable": False,
            "occupied": False,
            "status": "unknown",
            "reasons": reasons,
        })

    other_states = getattr(vehicle, "other_state", {}) or {}
    other_done = getattr(vehicle, "other_is_all_done", {}) or {}
    for vehicle_id, state in other_states.items():
        if bool(other_done.get(vehicle_id, False)):
            continue
        idx, dist = nearest_spot_index(spaces, [state.x.x, state.x.y])
        if idx is None or dist > float(other_vehicle_spot_radius):
            continue
        row = statuses[idx]
        row["occupied_by_vehicle_ids"].append(int(vehicle_id))
        row["reasons"].append("vehicle_%d_near_spot" % int(vehicle_id))

    for row in statuses:
        occupied = bool(row["central_occupied"] or row["occupied_by_vehicle_ids"] or row["reserved_by_vehicle_ids"])
        row["occupied"] = occupied
        row["selectable"] = bool(row["occupancy_known"] and not occupied)
        if not row["occupancy_known"]:
            row["status"] = "unknown"
        elif occupied:
            row["status"] = "occupied"
        else:
            row["status"] = "available"
    return statuses


def status_by_index(vehicle: Any) -> Dict[int, Dict[str, Any]]:
    return {int(row["spot_index"]): row for row in build_spot_statuses(vehicle)}


def effective_occupancy(vehicle: Any) -> List[int]:
    return [0 if row["selectable"] else 1 for row in build_spot_statuses(vehicle)]


def nearest_selectable_spot_indices(vehicle: Any, max_spots: int = 8) -> List[int]:
    rows = [row for row in build_spot_statuses(vehicle) if row["selectable"]]
    rows.sort(key=lambda row: row["distance"])
    return [int(row["spot_index"]) for row in rows[:max_spots]]


def nearest_spot_statuses(vehicle: Any, max_spots: int = 12) -> List[Dict[str, Any]]:
    rows = build_spot_statuses(vehicle)
    rows.sort(key=lambda row: row["distance"])
    return rows[:max_spots]
