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
    packet_json = json.dumps(packet, ensure_ascii=False, sort_keys=True)
    return (
        "You are the cloud VLA fleet coordinator for a mixed human-autonomous parking lot. "
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
        "fleet_decision_packet:\n" + packet_json
    )
