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
    metrics.update(collect_system_traffic_metrics(log_dir))
    return metrics



def collect_system_traffic_metrics(
    log_dir: Path,
    near_miss_radius: float = 3.0,
    collision_radius: float = 1.5,
    intent_conflict_radius: float = 6.0,
    ttc_horizon: float = 4.0,
    max_time_gap: float = 0.25,
) -> Dict[str, Any]:
    traces: List[Tuple[Path, List[Dict[str, Any]]]] = []
    roles: Dict[str, str] = {}
    agent_types: Dict[str, str] = {}
    intent_observable: Dict[str, bool] = {}
    completed = 0
    for summary_path in sorted(log_dir.glob("vehicle_*_summary.json")):
        try:
            summary = json.loads(summary_path.read_text())
        except Exception:
            continue
        vehicle_id = str(summary.get("vehicle_id", ""))
        if vehicle_id:
            roles[vehicle_id] = str(summary.get("vehicle_role") or "unknown")
            agent_types[vehicle_id] = str(summary.get("agent_type") or "unknown")
            intent_observable[vehicle_id] = bool(summary.get("intent_observable", True))
        if summary.get("completed") is True:
            completed += 1
    for trace_path in sorted(log_dir.glob("vehicle_*_trace.jsonl")):
        rows = load_jsonl(trace_path)
        if not rows:
            continue
        first = rows[0]
        vehicle_id = str(first.get("vehicle_id", ""))
        if vehicle_id:
            roles.setdefault(vehicle_id, str(first.get("vehicle_role") or "unknown"))
            agent_types.setdefault(vehicle_id, str(first.get("agent_type") or "unknown"))
            intent_observable.setdefault(vehicle_id, bool(first.get("intent_observable", True)))
        traces.append((trace_path, rows))

    pair_metrics = _pairwise_system_metrics(
        traces,
        roles,
        near_miss_radius=near_miss_radius,
        collision_radius=collision_radius,
        intent_conflict_radius=intent_conflict_radius,
        ttc_horizon=ttc_horizon,
        max_time_gap=max_time_gap,
    )
    traffic_events = load_jsonl(log_dir / "traffic_events.jsonl")
    schedule = _load_json(log_dir / "traffic_schedule.json")
    spawned = sum(1 for row in traffic_events if row.get("status") == "spawned")
    delayed = sum(1 for row in traffic_events if row.get("status") == "delayed")
    skipped = sum(1 for row in traffic_events if row.get("status") == "skipped")
    metrics = {
        "total_vehicle_trace_count": len(traces),
        "completed_vehicle_count": int(completed),
        "entering_vehicle_count": sum(1 for role in roles.values() if _is_entering_role(role)),
        "exiting_vehicle_count": sum(1 for role in roles.values() if _is_exiting_role(role)),
        "automated_vehicle_count": sum(1 for vehicle_id, role in roles.items() if _is_automated_vehicle(role, agent_types.get(vehicle_id, ""))),
        "cloud_served_vehicle_count": sum(1 for vehicle_id, role in roles.items() if _is_automated_vehicle(role, agent_types.get(vehicle_id, "")) and str(agent_types.get(vehicle_id, "")).lower() == "qwen_vla"),
        "human_like_vehicle_count": sum(1 for vehicle_id, role in roles.items() if not _is_automated_vehicle(role, agent_types.get(vehicle_id, ""))),
        "replay_vehicle_count": sum(1 for role in roles.values() if role == "replay_background"),
        "human_rule_vehicle_count": sum(1 for role in roles.values() if str(role).startswith("human_rule_")),
        "hidden_intent_vehicle_count": sum(1 for visible in intent_observable.values() if not visible),
        "traffic_scheduled_count": len(schedule.get("events", [])) if isinstance(schedule, dict) else 0,
        "traffic_hidden_event_count": int(schedule.get("hidden_event_count", 0)) if isinstance(schedule, dict) else 0,
        "traffic_spawned_count": int(spawned),
        "traffic_delayed_count": int(delayed),
        "traffic_skipped_count": int(skipped),
    }
    metrics.update(_fleet_completion_metrics(log_dir))
    metrics.update(pair_metrics)
    return metrics


def _fleet_completion_metrics(log_dir: Path) -> Dict[str, Any]:
    roles: Dict[str, str] = {}
    agent_types: Dict[str, str] = {}
    intent_observable: Dict[str, bool] = {}
    completed_by_id: Dict[str, bool] = {}
    summaries: Dict[str, Dict[str, Any]] = {}
    trace_by_id: Dict[str, List[Dict[str, Any]]] = {}
    controlled_summary_ids = set()
    controlled_trace_ids = set()
    for summary_path in sorted(log_dir.glob("vehicle_*_summary.json")):
        summary = _load_json(summary_path)
        vehicle_id = str(summary.get("vehicle_id", ""))
        if not vehicle_id:
            continue
        summaries[vehicle_id] = summary
        roles[vehicle_id] = str(summary.get("vehicle_role") or "unknown")
        agent_types[vehicle_id] = str(summary.get("agent_type") or "unknown")
        intent_observable[vehicle_id] = bool(summary.get("intent_observable", True))
        completed_by_id[vehicle_id] = bool(summary.get("completed") is True)
        if summary.get("is_controlled_ego") is True:
            controlled_summary_ids.add(vehicle_id)
    for trace_path in sorted(log_dir.glob("vehicle_*_trace.jsonl")):
        rows = load_jsonl(trace_path)
        if not rows:
            continue
        first = rows[0]
        final = rows[-1]
        vehicle_id = str(first.get("vehicle_id", final.get("vehicle_id", "")))
        if not vehicle_id:
            continue
        trace_by_id[vehicle_id] = rows
        roles.setdefault(vehicle_id, str(first.get("vehicle_role") or final.get("vehicle_role") or "unknown"))
        agent_types.setdefault(vehicle_id, str(first.get("agent_type") or final.get("agent_type") or "unknown"))
        intent_observable.setdefault(vehicle_id, bool(first.get("intent_observable", True)))
        if final.get("is_final") is True:
            completed_by_id[vehicle_id] = True
        if first.get("is_controlled_ego") is True or final.get("is_controlled_ego") is True:
            controlled_trace_ids.add(vehicle_id)
    vehicle_ids = set(roles.keys()) | set(agent_types.keys()) | set(trace_by_id.keys())
    automated_ids = {vehicle_id for vehicle_id in vehicle_ids if _is_automated_vehicle(roles.get(vehicle_id, ""), agent_types.get(vehicle_id, ""))}
    cloud_ids = {vehicle_id for vehicle_id in automated_ids if str(agent_types.get(vehicle_id, "")).lower() == "qwen_vla"}
    human_like_ids = vehicle_ids - automated_ids
    result = {
        "automated_vehicle_count": len(automated_ids),
        "completed_automated_vehicle_count": _completed_count(automated_ids, completed_by_id),
        "automated_vehicle_completion_rate": _completion_rate(automated_ids, completed_by_id),
        "cloud_served_vehicle_count": len(cloud_ids),
        "completed_cloud_served_vehicle_count": _completed_count(cloud_ids, completed_by_id),
        "cloud_served_vehicle_completion_rate": _completion_rate(cloud_ids, completed_by_id),
        "human_like_vehicle_count": len(human_like_ids),
        "completed_human_like_vehicle_count": _completed_count(human_like_ids, completed_by_id),
        "human_like_vehicle_completion_rate": _completion_rate(human_like_ids, completed_by_id),
        "completed_vehicle_count": _completed_count(vehicle_ids, completed_by_id),
        "controlled_ego_summary_count": len(controlled_summary_ids),
        "controlled_ego_trace_count": len(controlled_trace_ids),
        "controlled_ego_completed_count": _completed_count(controlled_summary_ids | controlled_trace_ids, completed_by_id),
        "controlled_ego_trace_points": sum(len(trace_by_id.get(vehicle_id, [])) for vehicle_id in (controlled_summary_ids | controlled_trace_ids)),
    }
    result.update(_fleet_operational_metrics("automated", automated_ids, trace_by_id, summaries))
    result.update(_fleet_operational_metrics("cloud_served", cloud_ids, trace_by_id, summaries))
    result.update(_fleet_operational_metrics("human_like", human_like_ids, trace_by_id, summaries))
    return result


def _completed_count(vehicle_ids: set, completed_by_id: Dict[str, bool]) -> int:
    return sum(1 for vehicle_id in vehicle_ids if completed_by_id.get(str(vehicle_id), False))


def _completion_rate(vehicle_ids: set, completed_by_id: Dict[str, bool]) -> float:
    return _completed_count(vehicle_ids, completed_by_id) / len(vehicle_ids) if vehicle_ids else 0.0


def _fleet_operational_metrics(prefix: str, vehicle_ids: set, trace_by_id: Dict[str, List[Dict[str, Any]]], summaries: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
    traces = [trace_by_id[str(vehicle_id)] for vehicle_id in vehicle_ids if str(vehicle_id) in trace_by_id]
    path_lengths = [_trace_path_length(rows) for rows in traces]
    waiting_times = [_trace_interval(rows, lambda row: int(row.get("waiting_for", 0) or 0) > 0) for rows in traces]
    idle_times = [_trace_interval(rows, lambda row: row.get("task") == "IDLE") for rows in traces]
    non_idle_times = []
    total_times = []
    for vehicle_id in vehicle_ids:
        rows = trace_by_id.get(str(vehicle_id), [])
        summary = summaries.get(str(vehicle_id), {})
        non_idle_times.append(_safe_float(summary.get("total_non_idle_time"), _trace_interval(rows, lambda row: row.get("task") != "IDLE")))
        total_times.append(_safe_float(summary.get("total_time"), _trace_duration(rows)))
    return {
        "fleet_total_%s_path_length" % prefix: round(sum(path_lengths), 6),
        "fleet_mean_%s_path_length" % prefix: _mean(path_lengths),
        "fleet_total_%s_waiting_time" % prefix: round(sum(waiting_times), 6),
        "fleet_mean_%s_waiting_time" % prefix: _mean(waiting_times),
        "fleet_total_%s_idle_time" % prefix: round(sum(idle_times), 6),
        "fleet_mean_%s_idle_time" % prefix: _mean(idle_times),
        "fleet_total_%s_non_idle_time" % prefix: round(sum(non_idle_times), 6),
        "fleet_mean_%s_non_idle_time" % prefix: _mean(non_idle_times),
        "fleet_mean_%s_total_time" % prefix: _mean(total_times),
    }


def _trace_duration(rows: List[Dict[str, Any]]) -> float:
    if not rows:
        return 0.0
    return max(0.0, _safe_float(rows[-1].get("time")) - _safe_float(rows[0].get("time")))


def _trace_interval(rows: List[Dict[str, Any]], predicate) -> float:
    total = 0.0
    for idx, row in enumerate(rows[:-1]):
        if predicate(row):
            total += _row_dt(rows, idx)
    return round(total, 6)


def _trace_path_length(rows: List[Dict[str, Any]]) -> float:
    total = 0.0
    for prev, cur in zip(rows, rows[1:]):
        total += _xy_distance(prev, cur)
    return round(total, 6)


def _load_json(path: Path) -> Dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text())
    except Exception:
        return {}


def _pairwise_system_metrics(
    traces: List[Tuple[Path, List[Dict[str, Any]]]],
    roles: Dict[str, str],
    near_miss_radius: float,
    collision_radius: float,
    intent_conflict_radius: float,
    ttc_horizon: float,
    max_time_gap: float,
) -> Dict[str, Any]:
    if len(traces) < 2:
        return {
            "system_min_distance_m": None,
            "system_near_miss_event_count": 0,
            "system_collision_proxy_event_count": 0,
            "trajectory_conflict_event_count": 0,
            "mixed_intent_conflict_event_count": 0,
            "mixed_intent_conflict_time_s": 0.0,
        }
    min_distance = float("inf")
    near_events = 0
    collision_events = 0
    trajectory_events = 0
    mixed_intent_events = 0
    mixed_intent_time = 0.0
    for left_idx in range(len(traces)):
        left_path, left_rows = traces[left_idx]
        left_id = _vehicle_id_from_rows(left_rows, left_path)
        for right_idx in range(left_idx + 1, len(traces)):
            right_path, right_rows = traces[right_idx]
            right_id = _vehicle_id_from_rows(right_rows, right_path)
            right_cursor = 0
            near_active = False
            collision_active = False
            trajectory_active = False
            mixed_active = False
            for row_idx, left_row in enumerate(left_rows):
                t = _sync_time(left_row)
                right_row, right_cursor = _nearest_time_row(right_rows, t, right_cursor)
                if right_row is None or abs(_sync_time(right_row) - t) > max_time_gap:
                    continue
                dt = _row_dt(left_rows, row_idx)
                distance = _xy_distance(left_row, right_row)
                min_distance = min(min_distance, distance)
                near = distance < near_miss_radius
                collision = distance < collision_radius
                ttc = _time_to_collision(left_row, right_row, horizon=ttc_horizon, radius=near_miss_radius)
                trajectory = ttc is not None
                mixed_intent = _is_mixed_enter_exit_pair(roles.get(left_id, ""), roles.get(right_id, "")) and distance < intent_conflict_radius
                if near and not near_active:
                    near_events += 1
                if collision and not collision_active:
                    collision_events += 1
                if trajectory and not trajectory_active:
                    trajectory_events += 1
                if mixed_intent:
                    mixed_intent_time += dt
                    if not mixed_active:
                        mixed_intent_events += 1
                near_active = near
                collision_active = collision
                trajectory_active = trajectory
                mixed_active = mixed_intent
    return {
        "system_min_distance_m": round(min_distance, 3) if math.isfinite(min_distance) else None,
        "system_near_miss_event_count": int(near_events),
        "system_collision_proxy_event_count": int(collision_events),
        "trajectory_conflict_event_count": int(trajectory_events),
        "mixed_intent_conflict_event_count": int(mixed_intent_events),
        "mixed_intent_conflict_time_s": round(mixed_intent_time, 3),
    }


def _vehicle_id_from_rows(rows: List[Dict[str, Any]], path: Path) -> str:
    if rows and rows[0].get("vehicle_id") is not None:
        return str(rows[0].get("vehicle_id"))
    name = path.name
    if name.startswith("vehicle_"):
        return name.split("_")[1]
    return name


def _is_mixed_enter_exit_pair(left_role: str, right_role: str) -> bool:
    return bool((_is_entering_role(left_role) and _is_exiting_role(right_role)) or (_is_exiting_role(left_role) and _is_entering_role(right_role)))


def _is_entering_role(role: str) -> bool:
    role = str(role)
    return role == "controlled_ego" or role.endswith("_entering") or role == "rule_entering"


def _is_exiting_role(role: str) -> bool:
    role = str(role)
    return role.endswith("_exiting") or role == "rule_exiting"


def _is_automated_agent(agent_type: str) -> bool:
    return str(agent_type).lower() in {
        "qwen_vla",
        "greedy_nearest",
        "greedy_shortest_path",
        "risk_aware_rule",
        "bundle_risk_aware",
        "conflict_aware_bundle",
        "min_bundle_cost",
        "reservation_bundle",
        "rolling_horizon_bundle",
        "centralized_min_cost",
        "oracle_intent_bundle",
        "vla_baseline",
    }


def _is_automated_vehicle(role: str, agent_type: str) -> bool:
    role = str(role)
    if role == "controlled_ego" or role.startswith("av_"):
        return True
    if role.startswith("human_") or role == "replay_background":
        return False
    return _is_automated_agent(agent_type)

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
    cloud_fleet_decisions = 0
    cloud_fleet_vehicle_decisions = 0
    cloud_fleet_missing_self_decisions = 0
    for row in decisions:
        fleet_packet = row.get("fleet_decision_packet") or {}
        fleet_response = row.get("fleet_response") or {}
        if fleet_packet:
            cloud_fleet_decisions += 1
            vehicle_ids = set(int(v) for v in fleet_packet.get("automated_vehicle_ids", []) if str(v).lstrip("-").isdigit())
            fleet_items = fleet_response.get("fleet_decisions", []) if isinstance(fleet_response, dict) else []
            if isinstance(fleet_items, list):
                cloud_fleet_vehicle_decisions += len(fleet_items)
                decided_ids = set()
                for item in fleet_items:
                    if not isinstance(item, dict):
                        continue
                    try:
                        decided_ids.add(int(item.get("vehicle_id")))
                    except Exception:
                        pass
                if vehicle_ids and not vehicle_ids.issubset(decided_ids):
                    cloud_fleet_missing_self_decisions += 1
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
        "cloud_fleet_decision_count": int(cloud_fleet_decisions),
        "cloud_fleet_vehicle_decision_count": int(cloud_fleet_vehicle_decisions),
        "cloud_fleet_missing_vehicle_decision_count": int(cloud_fleet_missing_self_decisions),
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
