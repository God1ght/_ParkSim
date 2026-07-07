from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional

import numpy as np

from parksim.rl.core import ParkSimCoreConfig, ParkSimCoreEnv
from parksim.rl.policy import flatten_observation
from parksim.rl.scenario import ParkSimScenarioLoader


@dataclass
class ExpertTransition:
    obs: Dict[str, np.ndarray]
    action: np.ndarray
    next_obs: Dict[str, np.ndarray]
    done: bool
    info: Dict[str, object]
    features: np.ndarray


@dataclass
class ExpertEpisode:
    agent_id: int
    transitions: List[ExpertTransition] = field(default_factory=list)
    metadata: Dict[str, object] = field(default_factory=dict)

    def __len__(self):
        return len(self.transitions)


class IRLFeatureExtractor:
    """Feature vector used by lightweight reward learning baselines."""

    feature_names = (
        "distance_to_goal",
        "heading_error",
        "speed_abs",
        "action_accel_abs",
        "action_steer_abs",
        "success",
        "collision",
    )

    def extract(
        self,
        obs: Dict[str, np.ndarray],
        action: np.ndarray,
        next_obs: Dict[str, np.ndarray],
        info: Dict[str, object],
    ) -> np.ndarray:
        reward_components = info.get("reward_components", {})
        achieved = next_obs["achieved_goal"]
        desired = next_obs["desired_goal"]
        distance = reward_components.get(
            "distance_to_goal",
            float(np.linalg.norm(achieved[:2] - desired[:2])),
        )
        heading_error = reward_components.get(
            "heading_error",
            float(abs((achieved[2] - desired[2] + np.pi) % (2 * np.pi) - np.pi)),
        )
        action = np.asarray(action, dtype=np.float32).reshape(-1)
        observation = np.asarray(next_obs["observation"], dtype=np.float32)
        speed = abs(float(observation[3])) if observation.shape[0] > 3 else 0.0
        return np.asarray(
            [
                distance,
                heading_error,
                speed,
                abs(float(action[0])) if action.shape[0] > 0 else 0.0,
                abs(float(action[1])) if action.shape[0] > 1 else 0.0,
                float(info.get("is_success", False)),
                float(info.get("collided", False)),
            ],
            dtype=np.float32,
        )


class LinearRewardModel:
    def __init__(self, weights: Iterable[float], bias: float = 0.0):
        self.weights = np.asarray(list(weights), dtype=np.float32)
        self.bias = float(bias)

    def predict(self, features: np.ndarray) -> float:
        features = np.asarray(features, dtype=np.float32)
        return float(features @ self.weights + self.bias)


class ExpertTrajectoryDataset:
    """Build expert episodes from private ParkSim data or deterministic fixtures."""

    def __init__(
        self,
        data_root: Optional[str] = None,
        allow_synthetic: bool = True,
        num_agents: int = 1,
        max_episode_steps: int = 160,
        seed: Optional[int] = None,
        feature_extractor: Optional[IRLFeatureExtractor] = None,
    ):
        self.data_root = data_root
        self.allow_synthetic = allow_synthetic
        self.num_agents = num_agents
        self.max_episode_steps = max_episode_steps
        self.seed = seed
        self.feature_extractor = feature_extractor or IRLFeatureExtractor()
        self.episodes: List[ExpertEpisode] = []

    def load(self, limit: Optional[int] = None) -> List[ExpertEpisode]:
        loader = ParkSimScenarioLoader(data_root=self.data_root, allow_synthetic=self.allow_synthetic)
        scenario = loader.load(seed=self.seed, num_agents=self.num_agents)
        config = ParkSimCoreConfig(
            num_agents=len(scenario.agents),
            max_steps=self.max_episode_steps,
            seed=self.seed,
            data_root=self.data_root,
            allow_synthetic_data=self.allow_synthetic,
        )
        env = ParkSimCoreEnv(config=config, scenario_loader=loader)
        observations, _ = env.reset(seed=self.seed, scenario=scenario)
        episodes = {agent_id: ExpertEpisode(agent_id=agent_id, metadata=dict(scenario.metadata)) for agent_id in env.agent_ids}
        max_steps = self.max_episode_steps if limit is None else min(self.max_episode_steps, limit)
        for _ in range(max_steps):
            actions = {
                agent_id: self._scripted_expert_action(env, agent_id)
                for agent_id in env.agent_ids
                if not env.done.get(agent_id, False)
            }
            if not actions:
                break
            next_observations, rewards, terminations, truncations, infos = env.step(actions)
            for agent_id, action in actions.items():
                done = bool(terminations[agent_id] or truncations[agent_id])
                features = self.feature_extractor.extract(
                    observations[agent_id],
                    action,
                    next_observations[agent_id],
                    infos[agent_id],
                )
                transition_info = dict(infos[agent_id])
                transition_info["reward"] = rewards[agent_id]
                episodes[agent_id].transitions.append(
                    ExpertTransition(
                        obs=observations[agent_id],
                        action=np.asarray(action, dtype=np.float32),
                        next_obs=next_observations[agent_id],
                        done=done,
                        info=transition_info,
                        features=features,
                    )
                )
            observations = next_observations
            if all(env.done.values()):
                break
        self.episodes = list(episodes.values())
        return self.episodes

    def transitions(self) -> List[ExpertTransition]:
        return [transition for episode in self.episodes for transition in episode.transitions]

    def feature_matrix(self) -> np.ndarray:
        transitions = self.transitions()
        if not transitions:
            return np.zeros((0, len(self.feature_extractor.feature_names)), dtype=np.float32)
        return np.vstack([transition.features for transition in transitions]).astype(np.float32)

    def as_dicts(self) -> List[Dict[str, object]]:
        rows = []
        for episode in self.episodes:
            for transition in episode.transitions:
                rows.append(
                    {
                        "agent_id": episode.agent_id,
                        "obs": transition.obs,
                        "action": transition.action,
                        "next_obs": transition.next_obs,
                        "done": transition.done,
                        "features": transition.features,
                        "info": transition.info,
                    }
                )
        return rows

    @staticmethod
    def _scripted_expert_action(env: ParkSimCoreEnv, agent_id: int) -> np.ndarray:
        state = env.states[agent_id]
        target = env.targets[agent_id]
        dx = float(target[0] - state.x.x)
        dy = float(target[1] - state.x.y)
        desired_yaw = np.arctan2(dy, dx)
        heading_error = (desired_yaw - state.e.psi + np.pi) % (2 * np.pi) - np.pi
        distance = np.linalg.norm([dx, dy])
        accel = np.clip(distance / 6.0 - abs(state.v.v) / 3.0, -1.0, 1.0)
        steer = np.clip(heading_error / 0.8, -1.0, 1.0)
        return np.asarray([accel, steer], dtype=np.float32)


def summarize_expert_dataset(dataset: ExpertTrajectoryDataset) -> Dict[str, object]:
    features = dataset.feature_matrix()
    return {
        "episodes": len(dataset.episodes),
        "transitions": len(dataset.transitions()),
        "feature_shape": features.shape,
        "feature_mean": features.mean(axis=0).tolist() if features.size else [],
        "feature_names": list(dataset.feature_extractor.feature_names),
    }


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Smoke-test ParkSim expert trajectory extraction.")
    parser.add_argument("--data-root", default=None)
    parser.add_argument("--num-agents", type=int, default=1)
    parser.add_argument("--steps", type=int, default=64)
    args = parser.parse_args()
    dataset = ExpertTrajectoryDataset(
        data_root=args.data_root,
        allow_synthetic=args.data_root is None,
        num_agents=args.num_agents,
        max_episode_steps=args.steps,
    )
    dataset.load()
    summary = summarize_expert_dataset(dataset)
    for key, value in summary.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
