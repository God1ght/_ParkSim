from dataclasses import dataclass, field
from typing import Dict


@dataclass
class MetricsCollector:
    episode_return: Dict[int, float] = field(default_factory=dict)
    collisions: Dict[int, int] = field(default_factory=dict)
    successes: Dict[int, int] = field(default_factory=dict)
    steps: int = 0

    def reset(self, agent_ids):
        self.episode_return = {agent_id: 0.0 for agent_id in agent_ids}
        self.collisions = {agent_id: 0 for agent_id in agent_ids}
        self.successes = {agent_id: 0 for agent_id in agent_ids}
        self.steps = 0

    def update(self, rewards, infos):
        self.steps += 1
        for agent_id, reward in rewards.items():
            self.episode_return[agent_id] = self.episode_return.get(agent_id, 0.0) + float(reward)
            if infos[agent_id].get("collided", False):
                self.collisions[agent_id] = self.collisions.get(agent_id, 0) + 1
            if infos[agent_id].get("is_success", False):
                self.successes[agent_id] = self.successes.get(agent_id, 0) + 1

    def summary(self):
        return {
            "episode_return": dict(self.episode_return),
            "collisions": dict(self.collisions),
            "successes": dict(self.successes),
            "steps": self.steps,
        }
