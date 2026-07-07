from dataclasses import dataclass, field
from typing import Dict

import numpy as np

from parksim.pytypes import VehicleActuation, VehicleState


@dataclass
class RewardWeights:
    distance: float = -0.08
    heading: float = -0.02
    speed: float = -0.01
    control: float = -0.002
    time: float = -0.01
    collision: float = -10.0
    success: float = 20.0


@dataclass
class RewardCalculator:
    weights: RewardWeights = field(default_factory=RewardWeights)
    success_radius: float = 1.25
    success_heading_tolerance: float = 0.6

    def compute(
        self,
        state: VehicleState,
        target_xy: np.ndarray,
        target_yaw: float,
        action: VehicleActuation,
        collided: bool,
    ) -> Dict[str, float]:
        dist = float(np.linalg.norm([state.x.x - target_xy[0], state.x.y - target_xy[1]]))
        heading_error = float(abs(self._wrap(state.e.psi - target_yaw)))
        success = dist <= self.success_radius and heading_error <= self.success_heading_tolerance
        components = {
            "distance": self.weights.distance * dist,
            "heading": self.weights.heading * heading_error,
            "speed": self.weights.speed * abs(state.v.v),
            "control": self.weights.control * (abs(action.u_a) + abs(action.u_steer)),
            "time": self.weights.time,
            "collision": self.weights.collision if collided else 0.0,
            "success": self.weights.success if success else 0.0,
        }
        components["total"] = float(sum(components.values()))
        components["is_success"] = float(success)
        components["distance_to_goal"] = dist
        components["heading_error"] = heading_error
        return components

    @staticmethod
    def _wrap(angle: float) -> float:
        return (angle + np.pi) % (2 * np.pi) - np.pi
