from dataclasses import dataclass
from typing import Dict

import numpy as np

from parksim.pytypes import VehicleState


@dataclass
class ObservationBuilder:
    max_neighbors: int = 6
    neighbor_radius: float = 30.0
    include_occupancy: bool = True
    occupancy_size: int = 32

    def build(
        self,
        agent_id: int,
        states: Dict[int, VehicleState],
        targets: Dict[int, np.ndarray],
        target_yaws: Dict[int, float],
        occupancy: np.ndarray,
        step_count: int,
        max_steps: int,
    ) -> Dict[str, np.ndarray]:
        ego = states[agent_id]
        target_xy = np.asarray(targets[agent_id], dtype=np.float32)
        achieved_goal = np.asarray([ego.x.x, ego.x.y, ego.e.psi], dtype=np.float32)
        desired_goal = np.asarray([target_xy[0], target_xy[1], target_yaws[agent_id]], dtype=np.float32)
        rel_goal = np.asarray(
            [target_xy[0] - ego.x.x, target_xy[1] - ego.x.y, target_yaws[agent_id] - ego.e.psi],
            dtype=np.float32,
        )

        ego_vec = np.asarray(
            [
                ego.x.x,
                ego.x.y,
                ego.e.psi,
                ego.v.v,
                ego.u.u_a,
                ego.u.u_steer,
                step_count / max(max_steps, 1),
            ],
            dtype=np.float32,
        )
        neighbors = self._neighbors(agent_id, states)
        occ = self._occupancy(occupancy) if self.include_occupancy else np.zeros(0, dtype=np.float32)
        observation = np.concatenate([ego_vec, rel_goal, neighbors.reshape(-1), occ]).astype(np.float32)
        return {
            "observation": observation,
            "achieved_goal": achieved_goal,
            "desired_goal": desired_goal,
            "action_mask": np.ones(2, dtype=np.float32),
        }

    def observation_size(self) -> int:
        occ = self.occupancy_size if self.include_occupancy else 0
        return 7 + 3 + self.max_neighbors * 6 + occ

    def _neighbors(self, agent_id: int, states: Dict[int, VehicleState]) -> np.ndarray:
        ego = states[agent_id]
        rows = []
        for other_id, other in states.items():
            if other_id == agent_id:
                continue
            dx = other.x.x - ego.x.x
            dy = other.x.y - ego.x.y
            dist = np.linalg.norm([dx, dy])
            if dist <= self.neighbor_radius:
                rows.append([dx, dy, other.e.psi - ego.e.psi, other.v.v, other.u.u_a, other.u.u_steer])
        rows = sorted(rows, key=lambda row: np.linalg.norm(row[:2]))[: self.max_neighbors]
        padded = np.zeros((self.max_neighbors, 6), dtype=np.float32)
        if rows:
            padded[: len(rows)] = np.asarray(rows, dtype=np.float32)
        return padded

    def _occupancy(self, occupancy: np.ndarray) -> np.ndarray:
        occ = np.asarray(occupancy, dtype=np.float32).reshape(-1)
        out = np.zeros(self.occupancy_size, dtype=np.float32)
        n = min(self.occupancy_size, occ.shape[0])
        out[:n] = occ[:n]
        return out
