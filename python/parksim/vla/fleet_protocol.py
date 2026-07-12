import json
from pathlib import Path
from typing import Any, Dict

from parksim.vla.fleet_schema import VLAFleetContext, VLA_FLEET_HARD_CONSTRAINTS, VLA_FLEET_OUTPUT_SCHEMA
from parksim.vla.schema import VLA_REASON_CODES


def build_fleet_decision_packet(context: VLAFleetContext, include_bev_path: bool = True) -> Dict[str, Any]:
    context_dict = context.to_dict()
    bev_path = context.bev_image_path if include_bev_path else None
    bev_attached = bool(context.bev_image_path and Path(context.bev_image_path).exists())
    return {
        "protocol_version": context.protocol_version,
        "prompt_version": context.prompt_version,
        "instruction": context.instruction,
        "output_schema": dict(VLA_FLEET_OUTPUT_SCHEMA),
        "reason_codes": list(VLA_REASON_CODES),
        "hard_constraints": list(VLA_FLEET_HARD_CONSTRAINTS),
        "state": context_dict.get("state", {}),
        "automated_vehicle_ids": list(context_dict.get("automated_vehicle_ids", [])),
        "automated_vehicles": list(context_dict.get("automated_vehicles", [])),
        "bev_image": {
            "attached": bev_attached,
            "path": bev_path,
        },
    }


def build_fleet_qwen_prompt(packet: Dict[str, Any]) -> str:
    compact_packet = build_compact_fleet_prompt_packet(packet)
    packet_json = json.dumps(compact_packet, ensure_ascii=False, sort_keys=True)
    return (
        "You are the cloud multimodal-LLM fleet coordinator for a mixed human-autonomous parking lot. "
        "Read the fleet_decision_packet JSON and optional BEV image. Return exactly one strict JSON object. "
        "The JSON must contain fleet_decisions, with one item per automated_vehicle_ids entry. "
        "For each vehicle, copy action_id exactly from that vehicle's valid_action_ids and keep target_spot_index consistent with the selected action. "
        "Do not output markdown, free-form routes, low-level controls, speeds, steering, throttle, or brake. "
        "You may coordinate automated vehicles through valid wait/yield/reroute/spot-assignment actions only. "
        "Never assign occupied, blocked, unknown, non-selectable, or already reserved spots. "
        "Never assign the same parking target to two automated vehicles in one fleet decision. "
        "Replay, rule-random, and mixed human vehicles are not controllable by you; treat hidden human intent as uncertainty. "
        "Optimize system-level efficiency and safety: completion, wait time, path length, near-miss risk, trajectory conflicts, and reservation conflicts. "
        "Qwen latency is an online cost metric only and must not change simulation-time ordering.\n\n"
        "compact_fleet_decision_packet:\n" + packet_json
    )


def build_compact_fleet_prompt_packet(packet: Dict[str, Any]) -> Dict[str, Any]:
    """Retain executable choices while keeping the model context bounded for fleet batches."""
    state = packet.get("state", {}) if isinstance(packet.get("state"), dict) else {}
    world_model = state.get("world_model", {}) if isinstance(state.get("world_model"), dict) else {}
    vehicles = packet.get("automated_vehicles", [])
    if not isinstance(vehicles, list):
        vehicles = []
    compact_vehicles = []
    for vehicle in vehicles:
        if not isinstance(vehicle, dict):
            continue
        vehicle_state = vehicle.get("state", {}) if isinstance(vehicle.get("state"), dict) else {}
        ego = vehicle_state.get("ego", {}) if isinstance(vehicle_state.get("ego"), dict) else {}
        ego_state = ego.get("state", {}) if isinstance(ego.get("state"), dict) else {}
        compact_vehicles.append({
            "vehicle_id": vehicle.get("vehicle_id"),
            "task": ego.get("task"),
            "pose": {key: _rounded(ego_state.get(key)) for key in ("x", "y", "yaw", "speed")},
            "current_target": ego.get("target"),
            "valid_actions": _compact_actions(vehicle.get("valid_actions", [])),
        })
    human_states = state.get("observable_human_vehicle_states", [])
    human_beliefs = state.get("human_intent_beliefs", [])
    conflict_graph = state.get("candidate_conflict_graph", {}) if isinstance(state.get("candidate_conflict_graph"), dict) else {}
    outcome_memory = state.get("previous_outcome_summary", {}) if isinstance(state.get("previous_outcome_summary"), dict) else {}
    return {
        "protocol_version": packet.get("protocol_version"),
        "output_schema": packet.get("output_schema"),
        "reason_codes": packet.get("reason_codes", []),
        "hard_constraints": packet.get("hard_constraints", []),
        "scene": {
            "sim_time": _rounded(state.get("sim_time")),
            "available_spots": world_model.get("available_spots"),
            "blocked_or_unknown_spots": world_model.get("blocked_or_unknown_spots"),
            "human_vehicle_count": len(human_states) if isinstance(human_states, list) else 0,
            "human_intent_model": state.get("human_intent_model"),
            "human_intents_revealed": False,
            "human_intent_beliefs": list(human_beliefs)[:32] if isinstance(human_beliefs, list) else [],
            "candidate_conflict_edge_count": int(conflict_graph.get("edge_count", 0) or 0),
        },
        "candidate_conflict_graph": {
            "schema_version": conflict_graph.get("schema_version"),
            "safety_distance_m": conflict_graph.get("safety_distance_m"),
            "edges": list(conflict_graph.get("edges", []))[:96],
        },
        "previous_outcome_summary": outcome_memory,
        "automated_vehicle_ids": packet.get("automated_vehicle_ids", []),
        "automated_vehicles": compact_vehicles,
    }


def _compact_actions(actions: Any, max_progress: int = 6) -> list:
    if not isinstance(actions, list):
        return []
    progress = []
    waits = []
    exits = []
    for action in actions:
        if not isinstance(action, dict) or not action.get("action_id"):
            continue
        action_type = str(action.get("action_type", ""))
        if action_type == "WAIT":
            waits.append(action)
        elif action_type == "CRUISE_TO_EXIT":
            exits.append(action)
        else:
            progress.append(action)
    selected = sorted(progress, key=_action_rank)[:max_progress]
    if waits:
        selected.append(min(waits, key=_action_rank))
    if exits:
        selected.append(min(exits, key=_action_rank))
    result = []
    seen = set()
    for action in selected:
        action_id = str(action.get("action_id"))
        if action_id in seen:
            continue
        seen.add(action_id)
        features = action.get("features", {}) if isinstance(action.get("features"), dict) else {}
        result.append({
            "action_id": action_id,
            "action_type": action.get("action_type"),
            "target_spot_index": action.get("target_spot_index"),
            "route_id": action.get("route_id"),
            "bundle_cost": _rounded(features.get("bundle_cost")),
            "conflict_risk": _rounded(features.get("conflict_risk")),
            "estimated_time_s": _rounded(features.get("estimated_time_s")),
            "expected_wait_s": _rounded(features.get("expected_wait_s")),
            "route_start_delay_s": _rounded(features.get("route_start_delay_s")),
            "route_end_time_s": _rounded(features.get("route_end_time_s")),
        })
    return result


def _action_rank(action: Dict[str, Any]) -> tuple:
    features = action.get("features", {}) if isinstance(action.get("features"), dict) else {}
    try:
        cost = float(features.get("bundle_cost"))
    except Exception:
        cost = float("inf")
    try:
        risk = float(features.get("conflict_risk"))
    except Exception:
        risk = float("inf")
    return cost, risk, str(action.get("action_id", ""))


def _rounded(value: Any) -> Any:
    try:
        return round(float(value), 3)
    except Exception:
        return value
