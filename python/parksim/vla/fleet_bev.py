"""Parking-lot-level BEV evidence for synchronized cloud VLA epochs."""
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import numpy as np

from parksim.vla.schema import VLAContext


def save_fleet_bev_png(contexts: Iterable[VLAContext], path: str, image_size: Tuple[int, int] = (768, 768)) -> str:
    image = render_fleet_bev_array(contexts, image_size=image_size)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image

        Image.fromarray(image).save(str(out))
    except Exception:
        fallback = out.with_suffix(".npy")
        np.save(str(fallback), image)
        return str(fallback)
    return str(out)


def render_fleet_bev_array(contexts: Iterable[VLAContext], image_size: Tuple[int, int] = (768, 768)) -> np.ndarray:
    contexts = list(contexts)
    width, height = image_size
    image = np.full((height, width, 3), 246, dtype=np.uint8)
    spots: Dict[int, Dict[str, Any]] = {}
    automated: List[Dict[str, float]] = []
    humans: Dict[int, Dict[str, Any]] = {}

    for context in contexts:
        state = context.state if isinstance(context.state, dict) else {}
        ego = state.get("ego") or {}
        ego_state = ego.get("state") or {}
        automated.append({"x": _number(ego_state.get("x")), "y": _number(ego_state.get("y"))})
        for row in list(state.get("candidate_spots") or []) + list(state.get("blocked_nearby_spots") or []):
            if isinstance(row, dict) and row.get("spot_index") is not None and row.get("xy") is not None:
                spots[int(row["spot_index"])] = dict(row)
        for row in state.get("nearby_vehicles") or []:
            if isinstance(row, dict) and row.get("vehicle_id") is not None:
                humans[int(row["vehicle_id"])] = dict(row)

    points = [(item["x"], item["y"]) for item in automated]
    points.extend(tuple(_xy(row.get("xy"))) for row in spots.values())
    points.extend(tuple(_xy((row.get("state") or {}))) for row in humans.values())
    low, high = _bounds(points)

    def to_px(xy: Iterable[float]) -> Tuple[int, int]:
        x, y = _xy(xy)
        span = max(high[0] - low[0], high[1] - low[1], 1.0)
        scale = 0.82 * min(width, height) / span
        center_x = (low[0] + high[0]) / 2.0
        center_y = (low[1] + high[1]) / 2.0
        return int(width / 2.0 + (x - center_x) * scale), int(height / 2.0 - (y - center_y) * scale)

    for row in spots.values():
        status = str(row.get("status") or "unknown")
        color = (176, 210, 176) if status == "available" else (80, 80, 80) if status == "occupied" else (220, 190, 120)
        _draw_square(image, *to_px(row.get("xy")), radius=5, color=color)
    for row in humans.values():
        _draw_square(image, *to_px((row.get("state") or {})), radius=7, color=(215, 65, 65))
    for row in automated:
        _draw_square(image, *to_px(row), radius=9, color=(35, 125, 225))
    return image


def _bounds(points: List[Tuple[float, float]]) -> Tuple[np.ndarray, np.ndarray]:
    if not points:
        return np.array([-20.0, -20.0]), np.array([20.0, 20.0])
    array = np.asarray(points, dtype=float)
    low = np.min(array, axis=0) - 8.0
    high = np.max(array, axis=0) + 8.0
    return low, high


def _xy(value: Any) -> Tuple[float, float]:
    if isinstance(value, dict):
        return _number(value.get("x")), _number(value.get("y"))
    values = list(value or [0.0, 0.0])
    return _number(values[0] if values else 0.0), _number(values[1] if len(values) > 1 else 0.0)


def _number(value: Any) -> float:
    try:
        return float(value)
    except Exception:
        return 0.0


def _draw_square(image: np.ndarray, px: int, py: int, radius: int, color: Tuple[int, int, int]) -> None:
    height, width = image.shape[:2]
    x0, x1 = max(0, px - radius), min(width, px + radius + 1)
    y0, y1 = max(0, py - radius), min(height, py + radius + 1)
    if x0 < x1 and y0 < y1:
        image[y0:y1, x0:x1] = np.asarray(color, dtype=np.uint8)
