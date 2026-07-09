from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from parksim.vla.schema import (
    VLA_HARD_CONSTRAINTS,
    VLA_OUTPUT_SCHEMA,
    VLA_REASON_CODES,
    VLACandidateAction,
    VLAContext,
    VLADecision,
)


VLA_FLEET_DECISION_PROTOCOL_VERSION = "ParkSim-Qwen-VLA-Fleet-Decision-v1"
VLA_FLEET_PROMPT_VERSION = "qwen-vla-cloud-fleet-policy-v1"

VLA_FLEET_OUTPUT_SCHEMA = {
    "type": "object",
    "required": ["fleet_decisions"],
    "additional_properties": False,
    "properties": {
        "fleet_decisions": [
            {
                "vehicle_id": "integer id copied from automated_vehicle_ids",
                "action_id": "string copied exactly from that vehicle's valid_action_ids",
                "target_spot_index": "integer target from the selected action, or null",
                "priority": "integer ordering; lower values execute/yield first",
                "reason_code": "one value from reason_codes",
                "reason": "short natural-language justification without hidden reasoning",
                "confidence": "float in [0.0, 1.0]",
            }
        ]
    },
}

VLA_FLEET_HARD_CONSTRAINTS = (
    "Return one decision for every vehicle_id in automated_vehicle_ids.",
    "Each action_id must be copied exactly from the valid_action_ids of the same automated vehicle.",
    "Never assign an occupied, blocked, unknown, reserved, or non-selectable parking spot.",
    "Never assign the same target_spot_index to more than one automated vehicle in the same fleet_decision.",
    "Never create free-form routes, low-level control, steering, throttle, brake, or speed commands.",
    "Use wait/yield actions when two automated vehicles would otherwise conflict in a bottleneck or target the same spot.",
    "Treat replay/rule/human vehicle destinations as hidden unless explicitly marked observable.",
    "Optimize parking-lot system efficiency and safety, not only the nearest vehicle's progress.",
) + tuple(VLA_HARD_CONSTRAINTS)


@dataclass
class VLAFleetDecision:
    vehicle_id: int
    action_id: str
    target_spot_index: Optional[int] = None
    priority: int = 0
    reason_code: str = ""
    reason: str = ""
    confidence: float = 0.0
    raw_response: str = ""
    used_fallback: bool = False

    def to_dict(self) -> Dict[str, Any]:
        return {
            "vehicle_id": int(self.vehicle_id),
            "action_id": self.action_id,
            "target_spot_index": self.target_spot_index,
            "priority": int(self.priority),
            "reason_code": self.reason_code,
            "reason": self.reason,
            "confidence": float(self.confidence),
            "raw_response": self.raw_response,
            "used_fallback": bool(self.used_fallback),
        }

    def to_vla_decision(self) -> VLADecision:
        return VLADecision(
            action_id=self.action_id,
            target_spot_index=self.target_spot_index,
            reason_code=self.reason_code,
            reason=self.reason,
            confidence=self.confidence,
            raw_response=self.raw_response,
            used_fallback=self.used_fallback,
        )


@dataclass
class VLAFleetResponse:
    decisions: List[VLAFleetDecision] = field(default_factory=list)
    raw_response: str = ""

    def decision_for(self, vehicle_id: int) -> Optional[VLAFleetDecision]:
        for decision in self.decisions:
            if int(decision.vehicle_id) == int(vehicle_id):
                return decision
        return None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "fleet_decisions": [decision.to_dict() for decision in self.decisions],
            "raw_response": self.raw_response,
        }


@dataclass
class VLAFleetContext:
    instruction: str
    state: Dict[str, Any]
    vehicle_contexts: List[VLAContext] = field(default_factory=list)
    bev_image_path: Optional[str] = None
    protocol_version: str = VLA_FLEET_DECISION_PROTOCOL_VERSION
    prompt_version: str = VLA_FLEET_PROMPT_VERSION

    def to_dict(self) -> Dict[str, Any]:
        vehicles: List[Dict[str, Any]] = []
        automated_vehicle_ids: List[int] = []
        for context in self.vehicle_contexts:
            context_dict = context.to_dict()
            state = context_dict.get("state", {})
            ego = state.get("ego", {}) if isinstance(state, dict) else {}
            vehicle_id = int(ego.get("vehicle_id", -1))
            automated_vehicle_ids.append(vehicle_id)
            vehicles.append({
                "vehicle_id": vehicle_id,
                "instruction": context.instruction,
                "state": context_dict.get("state", {}),
                "valid_action_ids": context_dict.get("valid_action_ids", []),
                "valid_actions": context_dict.get("valid_actions", []),
            })
        return {
            "protocol_version": self.protocol_version,
            "prompt_version": self.prompt_version,
            "instruction": self.instruction,
            "output_schema": dict(VLA_FLEET_OUTPUT_SCHEMA),
            "single_vehicle_output_schema": dict(VLA_OUTPUT_SCHEMA),
            "reason_codes": list(VLA_REASON_CODES),
            "hard_constraints": list(VLA_FLEET_HARD_CONSTRAINTS),
            "state": dict(self.state or {}),
            "automated_vehicle_ids": automated_vehicle_ids,
            "automated_vehicles": vehicles,
            "bev_image_path": self.bev_image_path,
        }


def fleet_context_for_single_vehicle(context: VLAContext) -> VLAFleetContext:
    state = context.state if isinstance(context.state, dict) else {}
    ego = state.get("ego", {}) if isinstance(state, dict) else {}
    vehicle_id = int(ego.get("vehicle_id", -1))
    return VLAFleetContext(
        instruction=(
            "Act as the cloud VLA fleet coordinator for mixed human-autonomous parking operations. "
            "The current request contains the automated vehicle that needs an immediate high-level decision; "
            "nearby replay/rule/human vehicles are observable background traffic, not controllable by Qwen. "
            "Return fleet_decisions with exactly one valid high-level action for vehicle_id %d."
        ) % vehicle_id,
        state={
            "cloud_policy_role": "fleet_level_qwen_vla_server",
            "decision_scope": "shared_cloud_endpoint_single_vehicle_trigger",
            "controlled_automated_vehicle_count": 1,
            "human_vehicle_control": "replay_rule_random_mixed_only",
            "sim_time_alignment": "latency is logged but does not advance simulation time",
        },
        vehicle_contexts=[context],
        bev_image_path=context.bev_image_path,
    )
