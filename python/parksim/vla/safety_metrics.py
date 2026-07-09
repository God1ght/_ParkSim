import bisect
import json
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def load_jsonl(path: Path) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    if not path.exists():
        return rows
    with path.open() as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def collect_safety_metrics(
    log_dir: Path,
    ego_trace_path: Path,
    ego_trace: List[Dict[str, Any]],
    decisions: List[Dict[str, Any]],
    near_miss_radius: float = 3.0,
    collision_radius: float = 1.5,
    max_time_gap: float = 0.25,
) -> Dict[str, Any]:
    other_traces = _load_other_traces(log_dir, ego_trace_path)
    distance_metrics = _distance_safety_metrics(
        ego_trace,
        other_traces,
        near_miss_radius=near_miss_radius,
        collision_radius=collision_radius,
        max_time_gap=max_time_gap,
    )
    decision_metrics = _decision_safety_metrics(decisions)
    metrics = {
        "other_vehicle_trace_count": len(other_traces),
        "near_miss_radius_m": float(near_miss_radius),
        "collision_proxy_radius_m": float(collision_radius),
    }
    metrics.update(distance_metrics)
    metrics.update(decision_metrics)
    return metrics


def _load_other_traces(log_dir: Path, ego_trace_path: Path) -> List[Tuple[Path, List[Dict[str, Any]]]]:
    traces: List[Tuple[Path, List[Dict[str, Any]]]] = []
    ego_trace_path = ego_trace_path.resolve()
    for path in sorted(log_dir.glob("vehicle_*_trace.jsonl")):
        try:
            if path.resolve() == ego_trace_path:
                continue
        except FileNotFoundError:
            continue
        rows = load_jsonl(path)
        if rows:
            traces.append((path, rows))
    return traces


def _distance_safety_metrics(
    ego_trace: List[Dict[str, Any]],
    other_traces: List[Tuple[Path, List[Dict[str, Any]]]],
    near_miss_radius: float,
    collision_radius: float,
    max_time_gap: float,
) -> Dict[str, Any]:
    if not ego_trace or not other_traces:
        return {
            "min_other_distance_m": None,
            "near_miss_time_s": 0.0,
            "near_miss_event_count": 0,
            "collision_proxy_time_s": 0.0,
            "collision_proxy_event_count": 0,
            "min_ttc_s": None,
        }

    other_indices = [0 for _ in other_traces]
    near_active = False
    collision_active = False
    near_events = 0
    collision_events = 0
    near_time = 0.0
    collision_time = 0.0
    min_distance = float("inf")
    min_ttc = float("inf")

    for idx, ego_row in enumerate(ego_trace):
        t = _sync_time(ego_row)
        dt = _row_dt(ego_trace, idx)
        row_near = False
        row_collision = False
        for trace_idx, (_, other_rows) in enumerate(other_traces):
            other_row, other_indices[trace_idx] = _nearest_time_row(other_rows, t, other_indices[trace_idx])
            if other_row is None:
                continue
            if abs(_sync_time(other_row) - t) > max_time_gap:
                continue
            distance = _xy_distance(ego_row, other_row)
            min_distance = min(min_distance, distance)
            if distance < near_miss_radius:
                row_near = True
            if distance < collision_radius:
                row_collision = True
            ttc = _time_to_collision(ego_row, other_row, horizon=5.0, radius=near_miss_radius)
            if ttc is not None:
                min_ttc = min(min_ttc, ttc)
        if row_near:
            near_time += dt
        if row_collision:
            collision_time += dt
        if row_near and not near_active:
            near_events += 1
        if row_collision and not collision_active:
            collision_events += 1
        near_active = row_near
        collision_active = row_collision

    return {
        "min_other_distance_m": round(min_distance, 3) if math.isfinite(min_distance) else None,
        "near_miss_time_s": round(near_time, 3),
        "near_miss_event_count": int(near_events),
        "collision_proxy_time_s": round(collision_time, 3),
        "collision_proxy_event_count": int(collision_events),
        "min_ttc_s": round(min_ttc, 3) if math.isfinite(min_ttc) else None,
    }


def _decision_safety_metrics(decisions: List[Dict[str, Any]]) -> Dict[str, Any]:
    unsafe_occupancy = 0
    malformed_action = 0
    target_mismatch = 0
    missing_reason_code = 0
    candidate_counts: List[int] = []
    available_counts: List[int] = []
    occupied_or_unknown_counts: List[int] = []
    for row in decisions:
        action = row.get("applied_action") or {}
        features = action.get("features") or {}
        if action.get("target_spot_index") is not None:
            selectable = features.get("selectable", True)
            occupancy_status = features.get("occupancy_status")
            if selectable is False or occupancy_status in ("occupied", "unknown", "blocked"):
                unsafe_occupancy += 1
        decision = row.get("decision") or {}
        if not decision.get("action_id"):
            malformed_action += 1
        elif not decision.get("reason_code"):
            missing_reason_code += 1
        shield_reason = str(row.get("shield_reason") or "")
        if "target_spot_index" in shield_reason and shield_reason != "ok":
            target_mismatch += 1
        valid_actions = ((row.get("context") or {}).get("valid_actions") or [])
        if valid_actions:
            candidate_counts.append(len(valid_actions))
            available = 0
            occupied_or_unknown = 0
            for candidate in valid_actions:
                c_features = candidate.get("features") or {}
                status = c_features.get("occupancy_status")
                if c_features.get("selectable", True) and status in ("available", None):
                    available += 1
                if status in ("occupied", "unknown", "blocked") or c_features.get("selectable") is False:
                    occupied_or_unknown += 1
            available_counts.append(available)
            occupied_or_unknown_counts.append(occupied_or_unknown)
    return {
        "unsafe_occupancy_action_count": int(unsafe_occupancy),
        "malformed_decision_count": int(malformed_action),
        "target_mismatch_decision_count": int(target_mismatch),
        "missing_reason_code_count": int(missing_reason_code),
        "candidate_action_count_mean": _mean(candidate_counts),
        "available_candidate_count_mean": _mean(available_counts),
        "blocked_candidate_count_mean": _mean(occupied_or_unknown_counts),
    }


def _nearest_time_row(rows: List[Dict[str, Any]], t: float, start_idx: int) -> Tuple[Optional[Dict[str, Any]], int]:
    if not rows:
        return None, start_idx
    idx = max(0, min(start_idx, len(rows) - 1))
    while idx + 1 < len(rows) and _sync_time(rows[idx + 1]) <= t:
        idx += 1
    candidates = [idx]
    if idx + 1 < len(rows):
        candidates.append(idx + 1)
    best = min(candidates, key=lambda i: abs(_sync_time(rows[i]) - t))
    return rows[best], best


def _sync_time(row: Dict[str, Any]) -> float:
    if row.get("wall_time") is not None:
        return _safe_float(row.get("wall_time"))
    return _safe_float(row.get("time"))


def _row_dt(rows: List[Dict[str, Any]], idx: int) -> float:
    if idx + 1 < len(rows):
        return max(0.0, _safe_float(rows[idx + 1].get("time")) - _safe_float(rows[idx].get("time")))
    if idx > 0:
        return max(0.0, _safe_float(rows[idx].get("time")) - _safe_float(rows[idx - 1].get("time")))
    return 0.0


def _xy_distance(a: Dict[str, Any], b: Dict[str, Any]) -> float:
    return math.hypot(_safe_float(a.get("x")) - _safe_float(b.get("x")), _safe_float(a.get("y")) - _safe_float(b.get("y")))


def _time_to_collision(ego: Dict[str, Any], other: Dict[str, Any], horizon: float, radius: float) -> Optional[float]:
    rx = _safe_float(other.get("x")) - _safe_float(ego.get("x"))
    ry = _safe_float(other.get("y")) - _safe_float(ego.get("y"))
    evx, evy = _velocity(ego)
    ovx, ovy = _velocity(other)
    vx = ovx - evx
    vy = ovy - evy
    vv = vx * vx + vy * vy
    if vv <= 1e-9:
        return None
    t = -((rx * vx) + (ry * vy)) / vv
    if t <= 0.0 or t > horizon:
        return None
    closest = math.hypot(rx + vx * t, ry + vy * t)
    if closest > radius:
        return None
    return float(t)


def _velocity(row: Dict[str, Any]) -> Tuple[float, float]:
    speed = _safe_float(row.get("speed"))
    yaw = _safe_float(row.get("yaw"))
    return speed * math.cos(yaw), speed * math.sin(yaw)


def _safe_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _mean(values: List[float]) -> float:
    return round(sum(values) / len(values), 3) if values else 0.0
