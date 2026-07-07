from dataclasses import dataclass
from typing import Dict, Optional

import numpy as np

from parksim.pytypes import VehicleActuation, VehicleState
from parksim.rl.actions import ActionMapper
from parksim.rl.collision import pairwise_collisions
from parksim.rl.metrics import MetricsCollector
from parksim.rl.observations import ObservationBuilder
from parksim.rl.rewards import RewardCalculator
from parksim.rl.scenario import ParkSimScenario, ParkSimScenarioLoader
from parksim.vehicle_types import VehicleBody, VehicleConfig


@dataclass
class ParkSimCoreConfig:
    num_agents: int = 1
    max_steps: int = 300
    seed: Optional[int] = None
    data_root: Optional[str] = None
    allow_synthetic_data: bool = True


class ParkSimCoreEnv:
    """Pure Python ParkSim RL core for high-throughput rollout."""

    def __init__(
        self,
        config: Optional[ParkSimCoreConfig] = None,
        vehicle_body: Optional[VehicleBody] = None,
        vehicle_config: Optional[VehicleConfig] = None,
        scenario_loader: Optional[ParkSimScenarioLoader] = None,
        observation_builder: Optional[ObservationBuilder] = None,
        reward_calculator: Optional[RewardCalculator] = None,
    ):
        self.config = config or ParkSimCoreConfig()
        self.vehicle_body = vehicle_body or VehicleBody()
        self.vehicle_config = vehicle_config or VehicleConfig()
        self.scenario_loader = scenario_loader or ParkSimScenarioLoader(
            data_root=self.config.data_root,
            allow_synthetic=self.config.allow_synthetic_data,
        )
        self.observation_builder = observation_builder or ObservationBuilder()
        self.reward_calculator = reward_calculator or RewardCalculator()
        self.action_mapper = ActionMapper(self.vehicle_config, self.vehicle_body)
        self.metrics = MetricsCollector()
        self.rng = np.random.default_rng(self.config.seed)
        self.scenario: Optional[ParkSimScenario] = None
        self.states: Dict[int, VehicleState] = {}
        self.targets: Dict[int, np.ndarray] = {}
        self.target_yaws: Dict[int, float] = {}
        self.actions: Dict[int, VehicleActuation] = {}
        self.done: Dict[int, bool] = {}
        self.step_count = 0

    @property
    def agent_ids(self):
        return list(self.states.keys())

    def reset(self, seed: Optional[int] = None, scenario: Optional[ParkSimScenario] = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        scenario_seed = seed if seed is not None else self.config.seed
        self.scenario = scenario or self.scenario_loader.load(seed=scenario_seed, num_agents=self.config.num_agents)
        self.states = {agent_id: spec.initial_state.copy() for agent_id, spec in self.scenario.agents.items()}
        self.targets = {agent_id: np.asarray(spec.target_xy, dtype=np.float32) for agent_id, spec in self.scenario.agents.items()}
        self.target_yaws = {agent_id: float(spec.target_yaw) for agent_id, spec in self.scenario.agents.items()}
        self.actions = {agent_id: VehicleActuation() for agent_id in self.states}
        self.done = {agent_id: False for agent_id in self.states}
        self.step_count = 0
        self.metrics.reset(self.agent_ids)
        return self.observations(), self._reset_info()

    def step(self, actions: Dict[int, np.ndarray]):
        if self.scenario is None:
            raise RuntimeError("ParkSimCoreEnv.step() called before reset().")
        self.step_count += 1
        for agent_id, action in actions.items():
            if agent_id in self.states and not self.done.get(agent_id, False):
                self.actions[agent_id] = self.action_mapper.step_state(self.states[agent_id], action)

        collided_ids = set()
        for agent_a, agent_b in pairwise_collisions(self.states, self.vehicle_body):
            collided_ids.add(agent_a)
            collided_ids.add(agent_b)

        observations = self.observations()
        rewards = {}
        terminations = {}
        truncations = {}
        infos = {}
        for agent_id, state in self.states.items():
            collided = agent_id in collided_ids
            reward_parts = self.reward_calculator.compute(
                state,
                self.targets[agent_id],
                self.target_yaws[agent_id],
                self.actions[agent_id],
                collided,
            )
            success = bool(reward_parts["is_success"])
            truncated = self.step_count >= self.config.max_steps
            terminated = bool(success or collided)
            self.done[agent_id] = self.done.get(agent_id, False) or terminated or truncated
            rewards[agent_id] = reward_parts["total"]
            terminations[agent_id] = terminated
            truncations[agent_id] = truncated
            infos[agent_id] = {
                "reward_components": reward_parts,
                "is_success": success,
                "collided": collided,
                "distance_to_goal": reward_parts["distance_to_goal"],
                "heading_error": reward_parts["heading_error"],
                "metrics": self.metrics.summary(),
            }
        self.metrics.update(rewards, infos)
        return observations, rewards, terminations, truncations, infos

    def observations(self) -> Dict[int, Dict[str, np.ndarray]]:
        occupancy = self.scenario.occupied if self.scenario is not None else np.zeros(0, dtype=np.int8)
        return {
            agent_id: self.observation_builder.build(
                agent_id=agent_id,
                states=self.states,
                targets=self.targets,
                target_yaws=self.target_yaws,
                occupancy=occupancy,
                step_count=self.step_count,
                max_steps=self.config.max_steps,
            )
            for agent_id in self.states
        }

    def state_vector(self) -> np.ndarray:
        rows = []
        for agent_id in self.agent_ids:
            state = self.states[agent_id]
            target = self.targets[agent_id]
            rows.append([agent_id, state.x.x, state.x.y, state.e.psi, state.v.v, target[0], target[1]])
        return np.asarray(rows, dtype=np.float32).reshape(-1)

    def _reset_info(self):
        return {
            agent_id: {
                "scenario": self.scenario.name if self.scenario else None,
                "target_xy": self.targets[agent_id].copy(),
                "target_yaw": self.target_yaws[agent_id],
            }
            for agent_id in self.states
        }
