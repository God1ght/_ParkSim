from dataclasses import dataclass, field
import os
from pathlib import Path
import pickle
from typing import Dict, List, Optional

import numpy as np

from parksim.pytypes import VehicleState


@dataclass
class AgentScenario:
    agent_id: int
    initial_state: VehicleState
    target_xy: np.ndarray
    target_yaw: float = 0.0
    controlled: bool = True
    expert_task_profile: Optional[List[dict]] = None


@dataclass
class ParkSimScenario:
    parking_spaces: np.ndarray
    occupied: np.ndarray
    agents: Dict[int, AgentScenario] = field(default_factory=dict)
    name: str = "synthetic"
    metadata: Dict[str, object] = field(default_factory=dict)


class ParkSimScenarioLoader:
    """Load private ParkSim/DLP data when present, otherwise create tiny synthetic scenarios."""

    def __init__(self, data_root: Optional[str] = None, allow_synthetic: bool = True):
        configured_root = data_root or os.environ.get("PARKSIM_DATA_ROOT")
        self.data_root = Path(configured_root).expanduser() if configured_root else None
        self.allow_synthetic = allow_synthetic

    def load(self, seed: Optional[int] = None, num_agents: int = 1) -> ParkSimScenario:
        if self.data_root is not None:
            return self._load_private(seed=seed, num_agents=num_agents)
        if self.allow_synthetic:
            return self.synthetic(seed=seed, num_agents=num_agents)
        raise FileNotFoundError(
            "ParkSim private data is not configured. Set PARKSIM_DATA_ROOT to a directory containing "
            "spots_data.pickle and optional agents_data_0012.pickle, or enable allow_synthetic."
        )

    def _load_private(self, seed: Optional[int], num_agents: int) -> ParkSimScenario:
        root = self.data_root
        spots_path = root / "spots_data.pickle"
        agents_path = root / "agents_data_0012.pickle"
        if not spots_path.exists():
            raise FileNotFoundError(
                f"Missing {spots_path}. PARKSIM_DATA_ROOT must point at ParkSim priorFiles or an equivalent data folder."
            )

        with spots_path.open("rb") as f:
            spots_data = pickle.load(f)
        parking_spaces = np.asarray(spots_data["parking_spaces"], dtype=np.float32)
        occupied = np.zeros(len(parking_spaces), dtype=np.int8)

        rng = np.random.default_rng(seed)
        agents = {}
        if agents_path.exists():
            with agents_path.open("rb") as f:
                expert_agents = pickle.load(f)
            for idx, agent_id in enumerate(list(expert_agents.keys())[:num_agents]):
                raw = expert_agents[agent_id]
                state = VehicleState(vehicle_id=int(agent_id) + 1)
                if "init_coords" in raw:
                    state.x.x = float(raw["init_coords"][0])
                    state.x.y = float(raw["init_coords"][1])
                elif "init_spot" in raw:
                    spot = int(raw["init_spot"])
                    state.x.x = float(parking_spaces[spot][0])
                    state.x.y = float(parking_spaces[spot][1])
                state.e.psi = float(raw.get("init_heading", 0.0))
                state.v.v = float(raw.get("init_v", 0.0))
                target_spot = self._target_spot(raw, len(parking_spaces), rng)
                agents[idx] = AgentScenario(
                    agent_id=idx,
                    initial_state=state,
                    target_xy=np.asarray(parking_spaces[target_spot], dtype=np.float32),
                    target_yaw=0.0,
                    controlled=True,
                    expert_task_profile=raw.get("task_profile"),
                )
                occupied[target_spot] = 1
        else:
            return self.synthetic(seed=seed, num_agents=num_agents, parking_spaces=parking_spaces)

        return ParkSimScenario(
            parking_spaces=parking_spaces,
            occupied=occupied,
            agents=agents,
            name="parksim-private",
            metadata={"data_root": str(root), "agents_path": str(agents_path)},
        )

    def synthetic(
        self,
        seed: Optional[int] = None,
        num_agents: int = 1,
        parking_spaces: Optional[np.ndarray] = None,
    ) -> ParkSimScenario:
        rng = np.random.default_rng(seed)
        if parking_spaces is None:
            xs = np.linspace(-12.0, 12.0, 7)
            parking_spaces = np.asarray([[x, 0.0] for x in xs], dtype=np.float32)
        occupied = np.zeros(len(parking_spaces), dtype=np.int8)
        agents = {}
        for agent_id in range(num_agents):
            target_idx = int((agent_id * 2 + 2) % len(parking_spaces))
            state = VehicleState(vehicle_id=agent_id + 1)
            state.x.x = float(-8.0 + agent_id * 4.0 + rng.normal(0.0, 0.05))
            state.x.y = float(-14.0 - agent_id * 2.0)
            state.e.psi = float(np.pi / 2)
            state.v.v = 0.0
            agents[agent_id] = AgentScenario(
                agent_id=agent_id,
                initial_state=state,
                target_xy=np.asarray(parking_spaces[target_idx], dtype=np.float32),
                target_yaw=float(np.pi / 2),
                controlled=True,
            )
            occupied[target_idx] = 1
        return ParkSimScenario(
            parking_spaces=parking_spaces,
            occupied=occupied,
            agents=agents,
            name="synthetic",
            metadata={"seed": seed},
        )

    @staticmethod
    def _target_spot(raw: dict, num_spots: int, rng: np.random.Generator) -> int:
        for task in raw.get("task_profile", []):
            if "target_spot_index" in task and task["target_spot_index"] is not None:
                return int(abs(task["target_spot_index"])) % num_spots
        return int(rng.integers(0, num_spots))
