"""Observable-only human motion beliefs for fleet-level VLA decisions."""
from collections import defaultdict, deque
from dataclasses import dataclass
from typing import Any, Deque, Dict, Iterable, List, Tuple

import math


def _float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _wrap(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class MotionSample:
    sim_time: float
    x: float
    y: float
    yaw: float
    speed: float
    braking: bool


class HumanIntentBeliefTracker:
    """Tracks motion evidence without exposing a human target spot or reference path."""

    def __init__(self, history_seconds: float = 8.0, stale_seconds: float = 15.0) -> None:
        self.history_seconds = max(1.0, float(history_seconds))
        self.stale_seconds = max(self.history_seconds, float(stale_seconds))
        self._history: Dict[int, Deque[MotionSample]] = defaultdict(deque)
        self._operation: Dict[int, str] = {}
        self._last_seen: Dict[int, float] = {}

    def update(self, sim_time: float, observations: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
        now = float(sim_time)
        seen = set()
        for row in observations:
            if not isinstance(row, dict):
                continue
            try:
                vehicle_id = int(row.get("vehicle_id"))
            except Exception:
                continue
            state = row.get("state", {}) if isinstance(row.get("state"), dict) else {}
            sample = MotionSample(
                sim_time=now,
                x=_float(state.get("x")),
                y=_float(state.get("y")),
                yaw=_float(state.get("yaw")),
                speed=_float(state.get("speed")),
                braking=bool(row.get("is_braking", False)),
            )
            history = self._history[vehicle_id]
            if not history or now > history[-1].sim_time + 1e-9:
                history.append(sample)
            elif abs(now - history[-1].sim_time) <= 1e-9:
                history[-1] = sample
            while history and now - history[0].sim_time > self.history_seconds:
                history.popleft()
            operation = str(row.get("declared_operation", "unknown") or "unknown").lower()
            self._operation[vehicle_id] = operation if operation in ("entering", "exiting") else "unknown"
            self._last_seen[vehicle_id] = now
            seen.add(vehicle_id)

        for vehicle_id, last_seen in list(self._last_seen.items()):
            if now - last_seen > self.stale_seconds:
                self._last_seen.pop(vehicle_id, None)
                self._operation.pop(vehicle_id, None)
                self._history.pop(vehicle_id, None)

        return [self._belief(vehicle_id) for vehicle_id in sorted(seen)]

    def _belief(self, vehicle_id: int) -> Dict[str, Any]:
        history = list(self._history.get(vehicle_id, ()))
        current = history[-1]
        yaw_rate, acceleration = self._motion_rates(history)
        logits = {
            "continue_heading": 2.2 - 4.0 * abs(yaw_rate),
            "turn_left": 0.4 + 5.0 * max(0.0, yaw_rate),
            "turn_right": 0.4 + 5.0 * max(0.0, -yaw_rate),
            "stop_or_yield": 0.2 + (2.5 if current.braking else 0.0) + max(0.0, 1.2 - abs(current.speed)),
        }
        probabilities = self._softmax(logits)
        entropy = -sum(probability * math.log(max(probability, 1e-12)) for probability in probabilities.values())
        normalized_entropy = entropy / math.log(float(len(probabilities)))
        hypotheses = []
        for name, probability in sorted(probabilities.items(), key=lambda item: (-item[1], item[0])):
            hypotheses.append({
                "name": name,
                "probability": round(float(probability), 4),
                "predicted_polyline_xy": self._predict(current, name),
            })
        return {
            "vehicle_id": int(vehicle_id),
            "operation_class": self._operation.get(vehicle_id, "unknown"),
            "operation_class_observable": True,
            "target_spot_observable": False,
            "intended_route_observable": False,
            "history_sample_count": len(history),
            "history_window_s": round(max(0.0, history[-1].sim_time - history[0].sim_time), 3),
            "estimated_yaw_rate_rad_s": round(float(yaw_rate), 4),
            "estimated_acceleration_mps2": round(float(acceleration), 4),
            "normalized_entropy": round(float(normalized_entropy), 4),
            "motion_hypotheses": hypotheses,
        }

    @staticmethod
    def _motion_rates(history: List[MotionSample]) -> Tuple[float, float]:
        if len(history) < 2:
            return 0.0, 0.0
        previous, current = history[-2], history[-1]
        dt = max(1e-3, current.sim_time - previous.sim_time)
        return _wrap(current.yaw - previous.yaw) / dt, (current.speed - previous.speed) / dt

    @staticmethod
    def _softmax(logits: Dict[str, float]) -> Dict[str, float]:
        maximum = max(logits.values())
        weights = {key: math.exp(value - maximum) for key, value in logits.items()}
        total = sum(weights.values())
        return {key: value / total for key, value in weights.items()}

    @staticmethod
    def _predict(sample: MotionSample, hypothesis: str) -> List[List[float]]:
        turn_rate = 0.0
        speed = max(0.0, abs(sample.speed))
        if hypothesis == "turn_left":
            turn_rate = 0.22
        elif hypothesis == "turn_right":
            turn_rate = -0.22
        elif hypothesis == "stop_or_yield":
            speed *= 0.25
        x, y, yaw = sample.x, sample.y, sample.yaw
        output = [[round(x, 3), round(y, 3)]]
        for _ in range(4):
            yaw += turn_rate
            x += speed * math.cos(yaw)
            y += speed * math.sin(yaw)
            output.append([round(x, 3), round(y, 3)])
        return output
