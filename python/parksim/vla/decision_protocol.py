import json
from pathlib import Path
from typing import Any, Dict

from parksim.vla.schema import VLAContext, VLA_HARD_CONSTRAINTS, VLA_OUTPUT_SCHEMA, VLA_REASON_CODES


def build_decision_packet(context: VLAContext, include_bev_path: bool = True) -> Dict[str, Any]:
    """Return the exact structured decision input that a Qwen policy is allowed to use."""
    context_dict = context.to_dict()
    bev_path = context.bev_image_path if include_bev_path else None
    bev_attached = bool(context.bev_image_path and Path(context.bev_image_path).exists())
    return {
        "protocol_version": context.protocol_version,
        "prompt_version": context.prompt_version,
        "instruction": context.instruction,
        "output_schema": dict(VLA_OUTPUT_SCHEMA),
        "reason_codes": list(VLA_REASON_CODES),
        "hard_constraints": list(VLA_HARD_CONSTRAINTS),
        "state": context.state,
        "valid_action_ids": list(context_dict.get("valid_action_ids", [])),
        "valid_actions": list(context_dict.get("valid_actions", [])),
        "bev_image": {
            "attached": bev_attached,
            "path": bev_path,
        },
    }


def build_qwen_prompt(packet: Dict[str, Any]) -> str:
    packet_json = json.dumps(packet, ensure_ascii=False, sort_keys=True)
    return (
        "You are the high-level VLA policy for a parking-lot vehicle. "
        "Read the decision_packet JSON and optional BEV image. Return exactly one strict JSON object. "
        "The JSON must follow output_schema. The selected action_id must be copied exactly from valid_action_ids. "
        "Do not output markdown, commentary outside JSON, low-level controls, free-form routes, or invented parking spots. "
        "If a parking target is selected, target_spot_index must equal the selected valid action target. "
        "Blocked, occupied, unknown, or non-selectable spots are negative evidence and must never be selected.\n\n"
        "decision_packet:\n" + packet_json
    )
