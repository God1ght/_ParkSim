from typing import Optional

import numpy as np

from parksim.rl.core import ParkSimCoreConfig, ParkSimCoreEnv

try:
    import gymnasium as gym
    from gymnasium import spaces
except ImportError:  # pragma: no cover - exercised only when optional dependency is missing.
    gym = None
    spaces = None


class ParkSimParkingEnv(gym.Env if gym else object):
    """Gymnasium wrapper for a single controlled ParkSim parking agent."""

    metadata = {"render_modes": []}

    def __init__(self, config: Optional[ParkSimCoreConfig] = None):
        if gym is None:
            raise ImportError("ParkSimParkingEnv requires gymnasium. Install it to use the Gym wrapper.")
        self.core = ParkSimCoreEnv(config or ParkSimCoreConfig(num_agents=1))
        obs_size = self.core.observation_builder.observation_size()
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Dict(
            {
                "observation": spaces.Box(low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float32),
                "achieved_goal": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
                "desired_goal": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
                "action_mask": spaces.Box(low=0.0, high=1.0, shape=(2,), dtype=np.float32),
            }
        )
        self.agent_id = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        observations, infos = self.core.reset(seed=seed)
        self.agent_id = self.core.agent_ids[0]
        return observations[self.agent_id], infos[self.agent_id]

    def step(self, action):
        observations, rewards, terminations, truncations, infos = self.core.step({self.agent_id: action})
        return (
            observations[self.agent_id],
            float(rewards[self.agent_id]),
            bool(terminations[self.agent_id]),
            bool(truncations[self.agent_id]),
            infos[self.agent_id],
        )

    def render(self):
        return None


def make_vec_env(num_envs: int, config: Optional[ParkSimCoreConfig] = None, async_env: bool = False):
    if gym is None:
        raise ImportError("Vectorized ParkSim envs require gymnasium.")

    def factory(rank: int):
        def _make():
            base = config or ParkSimCoreConfig(num_agents=1)
            env_config = ParkSimCoreConfig(
                num_agents=base.num_agents,
                max_steps=base.max_steps,
                seed=None if base.seed is None else base.seed + rank,
                data_root=base.data_root,
                allow_synthetic_data=base.allow_synthetic_data,
            )
            return ParkSimParkingEnv(env_config)

        return _make

    env_fns = [factory(i) for i in range(num_envs)]
    vector_cls = gym.vector.AsyncVectorEnv if async_env else gym.vector.SyncVectorEnv
    return vector_cls(env_fns)
