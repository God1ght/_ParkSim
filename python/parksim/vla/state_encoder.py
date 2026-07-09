from typing import Any, Dict, Iterable, List, Optional

import numpy as np

from parksim.pytypes import VehicleState
from parksim.vla.schema import VLA_HARD_CONSTRAINTS, VLA_OUTPUT_SCHEMA, VLA_REASON_CODES
from parksim.vla.spot_status import build_spot_statuses, effective_occupancy, nearest_spot_statuses, occupancy_ready


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


def encode_nearby_vehicles(vehicle: Any, max_vehicles: int = 8) -> List[Dict[str, Any]]:
    rows = []
    reveal_intents = bool(getattr(vehicle, "reveal_background_intents_to_vla", False))
    for vehicle_id in sorted(getattr(vehicle, "other_state", {}).keys()):
        state = vehicle.other_state[vehicle_id]
        ego = vehicle.state
        dist = float(np.linalg.norm([state.x.x - ego.x.x, state.x.y - ego.x.y]))
        task = getattr(vehicle, "other_task", {}).get(vehicle_id)
        parking_progress = getattr(vehicle, "other_parking_progress", {}).get(vehicle_id)
        rows.append({
            "vehicle_id": int(vehicle_id),
            "distance": dist,
            "state": encode_vehicle_state(state),
            "intent_observable": bool(reveal_intents),
            "task": task if reveal_intents else "unknown",
            "parking_progress": parking_progress if reveal_intents else None,
            "hidden_intent_fields": [] if reveal_intents else ["task", "parking_progress", "target_or_destination"],
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
    all_statuses = build_spot_statuses(vehicle)
    available_count = sum(1 for row in all_statuses if row["selectable"])
    nearest_statuses = nearest_spot_statuses(vehicle, max_spots=max(max_spots * 4, 32))
    selectable_nearby = [row for row in nearest_statuses if row["selectable"]]
    blocked_nearby = [row for row in nearest_statuses if not row["selectable"]]
    valid_action_ids = [action.action_id for action in (valid_actions or [])]
    return {
        "decision_contract": {
            "output_schema": dict(VLA_OUTPUT_SCHEMA),
            "reason_codes": list(VLA_REASON_CODES),
            "hard_constraints": list(VLA_HARD_CONSTRAINTS),
            "input_evidence": [
                "candidate_spots are selectable verified parking spaces",
                "blocked_nearby_spots explain occupied, blocked, or unknown parking spaces",
                "valid_actions is the complete executable high-level action set",
                "nearby_vehicles and action features expose dynamic conflict risk",
                "background human/rule vehicle intent is partially observable by default",
                "BEV image is visual evidence only; structured valid_actions has priority",
            ],
            "spot_status_sources": [
                "central_occupancy from simulator static obstacles and rule-based reservations",
                "dynamic vehicle proximity to parking-space centers",
            ],
        },
        "observability_model": {
            "background_intents_revealed": bool(getattr(vehicle, "reveal_background_intents_to_vla", False)),
            "available_background_evidence": ["pose", "speed", "braking", "waiting_signal"],
            "hidden_background_evidence": [] if bool(getattr(vehicle, "reveal_background_intents_to_vla", False)) else ["destination", "planned_route", "task_profile"],
        },
        "world_model": {
            "occupancy_ready": occupancy_ready(vehicle),
            "num_spots": len(all_statuses),
            "available_spots": available_count,
            "blocked_or_unknown_spots": len(all_statuses) - available_count,
        },
        "ego": {
            "vehicle_id": int(getattr(vehicle, "vehicle_id", -1)),
            "task": getattr(vehicle, "current_task", None),
            "state": encode_vehicle_state(getattr(vehicle, "state", None)),
            "target": target,
            "is_braking": bool(getattr(vehicle, "is_braking", False)),
            "waiting_for": int(getattr(vehicle, "waiting_for", 0) or 0),
        },
        "candidate_spots": selectable_nearby[:max_spots],
        "blocked_nearby_spots": blocked_nearby[:max_spots],
        "nearby_vehicles": encode_nearby_vehicles(vehicle),
        "central_occupancy": encode_occupancy(getattr(vehicle, "occupancy", None), limit=128),
        "effective_occupancy": effective_occupancy(vehicle)[:128],
        "valid_action_ids": valid_action_ids,
    }
