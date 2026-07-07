from collections import deque
import array
from typing import Dict, Optional

import numpy as np

from parksim.agents.abstract_agent import AbstractAgent
from parksim.pytypes import VehiclePrediction, VehicleState
from parksim.rl.actions import ActionMapper
from parksim.rl.observations import ObservationBuilder
from parksim.rl.policy import PolicyAdapter, load_policy_adapter
from parksim.vehicle_types import VehicleBody, VehicleConfig, VehicleInfo, VehicleTask


class RLPolicyAgent(AbstractAgent):
    """ParkSim agent wrapper for learned policies used in ROS/GUI evaluation."""

    def __init__(
        self,
        vehicle_id: int,
        vehicle_body: Optional[VehicleBody] = None,
        vehicle_config: Optional[VehicleConfig] = None,
        policy: Optional[PolicyAdapter] = None,
        policy_path: Optional[str] = None,
        observation_builder: Optional[ObservationBuilder] = None,
        target_xy=None,
        target_yaw: float = 0.0,
        max_steps: int = 1000,
    ):
        self.vehicle_body = vehicle_body or VehicleBody()
        self.vehicle_config = vehicle_config or VehicleConfig()
        super().__init__(vehicle_id=vehicle_id, state=VehicleState(vehicle_id=vehicle_id), vehicle_body=self.vehicle_body)
        self.policy = policy or load_policy_adapter(policy_path)
        self.observation_builder = observation_builder or ObservationBuilder()
        self.action_mapper = ActionMapper(self.vehicle_config, self.vehicle_body)
        self.info = VehicleInfo()
        self.state_hist = []
        self.task_profile = []
        self.task_history = []
        self.current_task = "RL_POLICY"
        self.target_xy = np.asarray(target_xy if target_xy is not None else [0.0, 0.0], dtype=np.float32)
        self.target_yaw = float(target_yaw)
        self.max_steps = max_steps
        self.step_count = 0
        self.done = False
        self.parking_spaces = None
        self.occupancy = np.zeros(0, dtype=np.int8)
        self.logger = deque(maxlen=100)
        self.other_vehicles = set()
        self.nearby_vehicles = set()
        self.other_state: Dict[int, VehicleState] = {}
        self.other_ref_pose = {}
        self.other_ref_v = {}
        self.other_target_idx = {}
        self.other_priority = {}
        self.other_task = {}
        self.other_parking_progress = {}
        self.other_parking_start_time = {}
        self.other_is_braking = {}
        self.other_waiting_for = {}
        self.other_is_all_done = {}
        self.priority = 0
        self.waiting_for = 0
        self.is_braking = False

    def load_parking_spaces(self, spots_data_path: str):
        import pickle

        with open(spots_data_path, "rb") as f:
            data = pickle.load(f)
        self.parking_spaces = np.asarray(data["parking_spaces"], dtype=np.float32)

    def load_graph(self, waypoints_graph_path: str):
        return None

    def load_maneuver(self, offline_maneuver_path: str):
        return None

    def set_vehicle_state(self, state: VehicleState = None, spot_index: int = None, heading: float = None):
        if state is not None:
            self.state = state
            self.state.vehicle_id = self.vehicle_id
            return
        if spot_index is None:
            return
        if self.parking_spaces is None:
            raise RuntimeError("load_parking_spaces() must be called before set_vehicle_state(spot_index=...).")
        self.state.x.x = float(self.parking_spaces[spot_index][0])
        self.state.x.y = float(self.parking_spaces[spot_index][1])
        self.state.e.psi = float(heading if heading is not None else np.pi / 2)

    def set_task_profile(self, task_profile):
        self.task_profile = list(task_profile or [])
        for task in self.task_profile:
            if isinstance(task, VehicleTask) and task.target_spot_index is not None and self.parking_spaces is not None:
                self.target_xy = np.asarray(self.parking_spaces[abs(task.target_spot_index)], dtype=np.float32)
                self.target_yaw = np.pi / 2

    def execute_next_task(self):
        if self.task_profile:
            task = self.task_profile.pop(0)
            self.task_history.append(task)
            self.current_task = task.name
            if task.target_spot_index is not None and self.parking_spaces is not None:
                self.target_xy = np.asarray(self.parking_spaces[abs(task.target_spot_index)], dtype=np.float32)
            elif task.target_coords is not None:
                self.target_xy = np.asarray(task.target_coords, dtype=np.float32)
        else:
            self.current_task = "RL_POLICY"

    def get_other_info(self, active_vehicles: Dict[int, AbstractAgent]):
        active_ids = set([agent_id for agent_id in active_vehicles if agent_id != self.vehicle_id])
        self.other_vehicles = active_ids
        for agent_id in active_ids:
            other = active_vehicles[agent_id]
            self.other_state[agent_id] = other.state
            self.other_is_all_done[agent_id] = other.is_all_done()

    def get_central_occupancy(self, occupancy):
        self.occupancy = np.asarray(occupancy, dtype=np.int8)

    def solve(self, time=None):
        if self.done:
            return
        states = {0: self.state}
        for idx, other_id in enumerate(sorted(self.other_state.keys()), start=1):
            states[idx] = self.other_state[other_id]
        obs = self.observation_builder.build(
            agent_id=0,
            states=states,
            targets={0: self.target_xy},
            target_yaws={0: self.target_yaw},
            occupancy=self.occupancy,
            step_count=self.step_count,
            max_steps=self.max_steps,
        )
        action = self.policy.predict(obs, deterministic=True)
        self.action_mapper.step_state(self.state, action)
        self.step_count += 1
        self.state_hist.append(self.state.copy())
        self.logger.append(f"t = {time}: x = {self.state.x.x:.2f}, y = {self.state.x.y:.2f}")
        self.done = self.step_count >= self.max_steps or self._reached_target()

    def get_info(self):
        self.info.ref_pose = VehiclePrediction()
        self.info.ref_pose.x = array.array('d', [float(self.target_xy[0])])
        self.info.ref_pose.y = array.array('d', [float(self.target_xy[1])])
        self.info.ref_pose.psi = array.array('d', [float(self.target_yaw)])
        self.info.ref_v = 0.0
        self.info.target_idx = 0
        self.info.priority = self.priority
        self.info.task = self.current_task
        self.info.parking_progress = "RL_POLICY"
        self.info.is_braking = self.is_braking
        self.info.parking_start_time = float("inf")
        self.info.waiting_for = self.waiting_for
        self.info.disp_text = f"RL-{self.vehicle_id}"
        self.info.is_all_done = self.is_all_done()
        return self.info

    def is_all_done(self):
        return bool(self.done)

    def get_state_dict(self):
        return {
            "center-x": self.state.x.x,
            "center-y": self.state.x.y,
            "heading": self.state.e.psi,
            "corners": self.vehicle_body.V,
        }

    def get_other_vehicles(self):
        return dict(self.other_state)

    def _reached_target(self):
        dist = np.linalg.norm([self.state.x.x - self.target_xy[0], self.state.x.y - self.target_xy[1]])
        return bool(dist < 1.25)
