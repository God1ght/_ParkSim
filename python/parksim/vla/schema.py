from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


VLA_DECISION_PROTOCOL_VERSION = "ParkSim-Qwen-VLA-Decision-v1"
VLA_PROMPT_VERSION = "qwen-vla-high-level-policy-v1"

VLA_REASON_CODES = (
    "PARK_AVAILABLE",
    "AVOID_OCCUPIED_SPOT",
    "YIELD_TRAFFIC",
    "REROUTE_CONFLICT",
    "PARK_READY",
    "EXIT_READY",
    "FALLBACK_OR_RECOVERY",
    "EXECUTOR_PATH_RECOVERY",
)

VLA_OUTPUT_SCHEMA = {
    "type": "object",
    "required": ["action_id", "target_spot_index", "reason_code", "confidence"],
    "additional_properties": False,
    "properties": {
        "action_id": "string; exactly one value from valid_action_ids",
        "target_spot_index": "integer target from the selected action, or null for non-spot actions",
        "reason_code": "one value from reason_codes",
        "reason": "short natural-language justification without hidden reasoning",
        "confidence": "float in [0.0, 1.0]",
    },
}

VLA_HARD_CONSTRAINTS = (
    "Choose exactly one action_id listed in valid_action_ids.",
    "Never invent a parking spot, route, speed, steering angle, throttle, or brake command.",
    "Treat each valid action as an executable assignment-route-wait bundle; never modify its route_id, duration, or target.",
    "Never select a spot whose status is occupied, blocked, unknown, or not selectable.",
    "If target_spot_index is present, it must match the selected action target exactly.",
    "Use blocked_nearby_spots only as negative evidence; those spots are not valid targets.",
    "When candidate_assignment_bundles is present, prefer lower bundle_cost after satisfying all hard constraints.",
    "Background human/rule vehicles may hide destination and task intent; infer risk only from observable evidence and uncertainty fields.",
    "Prefer verified parking progress over waiting when a safe SELECT_SPOT_AND_CRUISE action exists.",
)


class VLAActionType:
    WAIT = "WAIT"
    SELECT_SPOT_AND_CRUISE = "SELECT_SPOT_AND_CRUISE"
    PARK = "PARK"
    REROUTE = "REROUTE"
    CRUISE_TO_EXIT = "CRUISE_TO_EXIT"


@dataclass
class VLACandidateAction:
    action_id: str
    action_type: str
    target_spot_index: Optional[int] = None
    target_coords: Optional[Any] = None
    duration: Optional[float] = None
    route_id: Optional[Any] = None
    reason: str = ""
    features: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        target_coords = self.target_coords
        if target_coords is not None:
            target_coords = np.asarray(target_coords, dtype=float).reshape(-1).tolist()
        return {
            "action_id": self.action_id,
            "action_type": self.action_type,
            "target_spot_index": self.target_spot_index,
            "target_coords": target_coords,
            "duration": self.duration,
            "route_id": self.route_id,
            "reason": self.reason,
            "features": dict(self.features or {}),
        }


@dataclass
class VLADecision:
    action_id: str
    reason: str = ""
    confidence: float = 0.0
    raw_response: str = ""
    used_fallback: bool = False
    target_spot_index: Optional[int] = None
    reason_code: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action_id": self.action_id,
            "target_spot_index": self.target_spot_index,
            "reason_code": self.reason_code,
            "reason": self.reason,
            "confidence": self.confidence,
            "raw_response": self.raw_response,
            "used_fallback": self.used_fallback,
        }


@dataclass
class VLAContext:
    instruction: str
    state: Dict[str, Any]
    valid_actions: List[VLACandidateAction] = field(default_factory=list)
    bev_image_path: Optional[str] = None
    protocol_version: str = VLA_DECISION_PROTOCOL_VERSION
    prompt_version: str = VLA_PROMPT_VERSION

    def to_dict(self) -> Dict[str, Any]:
        valid_actions = [action.to_dict() for action in self.valid_actions]
        return {
            "protocol_version": self.protocol_version,
            "prompt_version": self.prompt_version,
            "instruction": self.instruction,
            "output_schema": dict(VLA_OUTPUT_SCHEMA),
            "reason_codes": list(VLA_REASON_CODES),
            "hard_constraints": list(VLA_HARD_CONSTRAINTS),
            "state": self.state,
            "valid_action_ids": [action["action_id"] for action in valid_actions],
            "valid_actions": valid_actions,
            "bev_image_path": self.bev_image_path,
        }
