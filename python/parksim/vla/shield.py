from typing import Any, Dict, List, Optional, Tuple

from parksim.vla.schema import VLACandidateAction, VLADecision, VLAActionType


class VLASafetyShield:
    def validate(self, decision: VLADecision, valid_actions: List[VLACandidateAction], vehicle: Any = None) -> Tuple[bool, Optional[VLACandidateAction], str]:
        by_id: Dict[str, VLACandidateAction] = {action.action_id: action for action in valid_actions}
        action = by_id.get(decision.action_id)
        if action is None:
            return False, None, "action_id is not in valid_actions"
        if action.action_type in (VLAActionType.SELECT_SPOT_AND_CRUISE, VLAActionType.PARK, VLAActionType.REROUTE):
            if action.target_spot_index is None:
                return False, None, "spot action has no target_spot_index"
            if vehicle is not None and getattr(vehicle, "parking_spaces", None) is not None:
                spot_index = abs(int(action.target_spot_index))
                if spot_index >= len(vehicle.parking_spaces):
                    return False, None, "target_spot_index is outside parking space range"
            if vehicle is not None and action.action_type == VLAActionType.SELECT_SPOT_AND_CRUISE:
                occupancy = getattr(vehicle, "occupancy", None)
                if occupancy is not None:
                    occupancy = list(occupancy)
                    spot_index = abs(int(action.target_spot_index))
                    if spot_index < len(occupancy) and bool(occupancy[spot_index]):
                        return False, None, "target spot is already occupied"
        return True, action, "ok"
