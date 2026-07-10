from typing import Iterable, Set


REPLAY_BACKGROUND_MODES = {
    "replay",
    "mixed",
    "human_mixed",
    "human_mixed_long_horizon",
}


def replay_vehicle_ids(agent_ids: Iterable[object], background_mode: str) -> Set[int]:
    if str(background_mode).strip().lower() not in REPLAY_BACKGROUND_MODES:
        return set()
    result = set()
    for vehicle_id in agent_ids:
        try:
            result.add(int(vehicle_id))
        except (TypeError, ValueError):
            continue
    return result


class VehicleIdAllocator:
    """Allocate dynamic IDs above every replay ID and reject duplicate claims."""

    def __init__(self, reserved_ids: Iterable[int] = ()):
        self._reserved = {int(vehicle_id) for vehicle_id in reserved_ids}
        self._allocated = set()
        self._next_id = max(self._reserved | {0}) + 1

    @property
    def reserved_ids(self) -> Set[int]:
        return set(self._reserved)

    @property
    def allocated_ids(self) -> Set[int]:
        return set(self._allocated)

    def allocate(self) -> int:
        while self._next_id in self._reserved or self._next_id in self._allocated:
            self._next_id += 1
        vehicle_id = self._next_id
        self._allocated.add(vehicle_id)
        self._next_id += 1
        return vehicle_id

    def claim_reserved(self, vehicle_id: int) -> int:
        vehicle_id = int(vehicle_id)
        if vehicle_id in self._allocated:
            raise ValueError("vehicle id %d is already allocated" % vehicle_id)
        self._reserved.discard(vehicle_id)
        self._allocated.add(vehicle_id)
        return vehicle_id
