from dataclasses import dataclass
from typing import Iterable

import numpy as np

from parksim.controller.stanley_controller import normalize_angle
from parksim.pytypes import VehicleActuation, VehicleState
from parksim.vehicle_types import VehicleBody, VehicleConfig


@dataclass
class ActionMapper:
    """Map normalized RL actions to ParkSim actuation and bicycle dynamics."""

    vehicle_config: VehicleConfig
    vehicle_body: VehicleBody

    def denormalize(self, action: Iterable[float]) -> VehicleActuation:
        arr = np.asarray(action, dtype=np.float32).reshape(-1)
        if arr.shape[0] != 2:
            raise ValueError("ParkSim continuous action must have shape (2,): [acceleration, steering].")
        arr = np.clip(arr, -1.0, 1.0)
        acceleration = self._scale(arr[0], self.vehicle_config.a_min, self.vehicle_config.a_max)
        steering = self._scale(arr[1], self.vehicle_config.delta_min, self.vehicle_config.delta_max)
        return VehicleActuation(u_a=float(acceleration), u_steer=float(steering))

    def step_state(self, state: VehicleState, action: Iterable[float]) -> VehicleActuation:
        actuation = self.denormalize(action)
        self.apply_actuation(state, actuation)
        return actuation

    def apply_actuation(self, state: VehicleState, actuation: VehicleActuation) -> None:
        dt = self.vehicle_config.dt
        wheelbase = max(float(self.vehicle_body.wb), 1e-6)
        steering = float(np.clip(actuation.u_steer, self.vehicle_config.delta_min, self.vehicle_config.delta_max))
        acceleration = float(np.clip(actuation.u_a, self.vehicle_config.a_min, self.vehicle_config.a_max))

        state.u.u_a = acceleration
        state.u.u_steer = steering
        state.x.x += state.v.v * np.cos(state.e.psi) * dt
        state.x.y += state.v.v * np.sin(state.e.psi) * dt
        state.e.psi = normalize_angle(state.e.psi + state.v.v / wheelbase * np.tan(steering) * dt)
        state.v.v = float(np.clip(state.v.v + acceleration * dt, self.vehicle_config.v_min, self.vehicle_config.v_max))
        state.t = 0.0 if state.t is None else state.t + dt

    @staticmethod
    def _scale(value: float, low: float, high: float) -> float:
        return low + (float(value) + 1.0) * 0.5 * (high - low)
