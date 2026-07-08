from pathlib import Path
from typing import Any, Iterable, Tuple

import numpy as np

from parksim.vla.spot_status import build_spot_statuses


def render_bev_array(vehicle: Any, image_size: Tuple[int, int] = (768, 768), meters_per_pixel: float = 0.25) -> np.ndarray:
    width, height = image_size
    image = np.full((height, width, 3), 245, dtype=np.uint8)
    center = np.asarray([getattr(vehicle.state.x, "x", 0.0), getattr(vehicle.state.x, "y", 0.0)], dtype=float)

    def to_px(xy):
        xy = np.asarray(xy, dtype=float)
        delta = (xy - center) / float(meters_per_pixel)
        return int(width / 2 + delta[0]), int(height / 2 - delta[1])

    for row in build_spot_statuses(vehicle):
        px, py = to_px(row["xy"])
        if row["status"] == "available":
            color = (185, 215, 185)
        elif row["status"] == "occupied":
            color = (75, 75, 75)
        else:
            color = (210, 190, 120)
        _draw_square(image, px, py, 4, color)
    for _, other in getattr(vehicle, "other_state", {}).items():
        px, py = to_px([other.x.x, other.x.y])
        _draw_square(image, px, py, 7, (220, 60, 60))
    ego_px, ego_py = to_px([vehicle.state.x.x, vehicle.state.x.y])
    _draw_square(image, ego_px, ego_py, 9, (40, 120, 230))
    return image


def save_bev_png(vehicle: Any, path: str) -> str:
    image = render_bev_array(vehicle)
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image
        Image.fromarray(image).save(str(out))
    except Exception:
        np.save(str(out.with_suffix(".npy")), image)
        return str(out.with_suffix(".npy"))
    return str(out)


def _draw_square(image: np.ndarray, px: int, py: int, radius: int, color: Iterable[int]) -> None:
    h, w = image.shape[:2]
    x0, x1 = max(0, px - radius), min(w, px + radius + 1)
    y0, y1 = max(0, py - radius), min(h, py + radius + 1)
    if x0 < x1 and y0 < y1:
        image[y0:y1, x0:x1] = np.asarray(tuple(color), dtype=np.uint8)
