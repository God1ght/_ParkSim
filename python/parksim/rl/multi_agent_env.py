from typing import Dict, Optional

import numpy as np

from parksim.rl.core import ParkSimCoreConfig, ParkSimCoreEnv

try:
    from gymnasium import spaces
except ImportError:  # pragma: no cover - optional dependency.
    spaces = None

try:
    from pettingzoo.utils.env import ParallelEnv
except ImportError:  # pragma: no cover - optional dependency.
    ParallelEnv = object


class ParkSimParallelParkingEnv(ParallelEnv):
    """PettingZoo ParallelEnv wrapper for multi-vehicle ParkSim parking tasks."""

    metadata = {"name": "parksim_parallel_parking_v0", "render_modes": []}

    def __init__(self, config: Optional[ParkSimCoreConfig] = None):
        if spaces is None or ParallelEnv is object:
            raise ImportError(
                "ParkSimParallelParkingEnv requires pettingzoo and gymnasium. "
                "Install optional dependencies with `pip install -e .[rl]` from the python/ directory."
            )
        self.config = config or ParkSimCoreConfig(num_agents=2)
        if self.config.num_agents < 2:
            self.config.num_agents = 2
        self.core = ParkSimCoreEnv(self.config)
        self.possible_agents = [self._agent_name(i) for i in range(self.config.num_agents)]
        self.agents = []
        obs_size = self.core.observation_builder.observation_size()
        self._observation_spaces = {
            agent: spaces.Dict(
                {
                    "observation": spaces.Box(low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float32),
                    "achieved_goal": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
                    "desired_goal": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
                    "action_mask": spaces.Box(low=0.0, high=1.0, shape=(2,), dtype=np.float32),
                }
            )
            for agent in self.possible_agents
        }
        self._action_spaces = {
            agent: spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
            for agent in self.possible_agents
        }

    def reset(self, seed=None, options=None):
        observations, infos = self.core.reset(seed=seed)
        self.agents = [self._agent_name(agent_id) for agent_id in self.core.agent_ids]
        return self._name_dict(observations), self._name_dict(infos)

    def step(self, actions: Dict[str, np.ndarray]):
        id_actions = {self._agent_id(agent): action for agent, action in actions.items() if agent in self.agents}
        observations, rewards, terminations, truncations, infos = self.core.step(id_actions)
        named_obs = self._name_dict(observations)
        named_rewards = self._name_dict(rewards)
        named_terms = self._name_dict(terminations)
        named_truncs = self._name_dict(truncations)
        named_infos = self._name_dict(infos)
        self.agents = [
            agent
            for agent in self.agents
            if not (named_terms.get(agent, False) or named_truncs.get(agent, False))
        ]
        return named_obs, named_rewards, named_terms, named_truncs, named_infos

    def state(self):
        return self.core.state_vector()

    def observation_space(self, agent):
        return self._observation_spaces[agent]

    def action_space(self, agent):
        return self._action_spaces[agent]

    def render(self):
        return None

    @staticmethod
    def _agent_name(agent_id: int) -> str:
        return f"vehicle_{agent_id}"

    @staticmethod
    def _agent_id(agent_name: str) -> int:
        return int(agent_name.split("_")[-1])

    def _name_dict(self, values):
        return {self._agent_name(agent_id): value for agent_id, value in values.items()}
