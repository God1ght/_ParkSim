from itertools import combinations
from typing import Dict, Iterable, Tuple

import numpy as np

from parksim.pytypes import VehicleState
from parksim.utils.rectangle_to_circles import v2c
from parksim.vehicle_types import VehicleBody


def vehicles_collide(state_a: VehicleState, state_b: VehicleState, vehicle_body: VehicleBody) -> bool:
    circles_a = v2c(state_a, vehicle_body)
    circles_b = v2c(state_b, vehicle_body)
    for circle_a in circles_a:
        for circle_b in circles_b:
            dist = np.linalg.norm([circle_a[0] - circle_b[0], circle_a[1] - circle_b[1]])
            if dist < circle_a[2] + circle_b[2]:
                return True
    return False


def pairwise_collisions(
    states: Dict[int, VehicleState], vehicle_body: VehicleBody
) -> Iterable[Tuple[int, int]]:
    for agent_a, agent_b in combinations(states.keys(), 2):
        if vehicles_collide(states[agent_a], states[agent_b], vehicle_body):
            yield agent_a, agent_b
