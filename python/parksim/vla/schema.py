from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


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
    route_id: Optional[int] = None
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

    def to_dict(self) -> Dict[str, Any]:
        return {
            "action_id": self.action_id,
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

    def to_dict(self) -> Dict[str, Any]:
        return {
            "instruction": self.instruction,
            "state": self.state,
            "valid_actions": [action.to_dict() for action in self.valid_actions],
            "bev_image_path": self.bev_image_path,
        }
