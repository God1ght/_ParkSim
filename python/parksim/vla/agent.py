import json
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from parksim.agents.rule_based_stanley_vehicle import RuleBasedStanleyVehicle
from parksim.controller.stanley_controller import StanleyController
from parksim.controller_types import StanleyParams
from parksim.vehicle_types import VehicleBody, VehicleConfig
from parksim.vla.action_space import apply_candidate_action, build_candidate_actions, choose_default_action
from parksim.vla.bev_encoder import save_bev_png
from parksim.vla.qwen_client import QwenPolicyClient
from parksim.vla.schema import VLAContext, VLADecision
from parksim.vla.shield import VLASafetyShield
from parksim.vla.state_encoder import build_vla_state


class QwenVLAVehicle(RuleBasedStanleyVehicle):
    """High-level VLA policy wrapper over the existing rule-based executor."""

    def __init__(
        self,
        vehicle_id: int,
        vehicle_body: Optional[VehicleBody] = None,
        vehicle_config: Optional[VehicleConfig] = None,
        qwen_endpoint: str = "",
        qwen_model: str = "Qwen2.5-VL-7B-Instruct",
        qwen_timeout: float = 15.0,
        decision_period: float = 3.0,
        max_candidate_spots: int = 8,
        entrance_coords: Optional[Any] = None,
        fallback_spot_index: Optional[int] = None,
        periodic_replan: bool = False,
        decision_log_path: str = "",
    ):
        vehicle_body = vehicle_body or VehicleBody()
        vehicle_config = vehicle_config or VehicleConfig()
        controller_params = StanleyParams(dt=vehicle_config.dt)
        controller = StanleyController(control_params=controller_params, vehicle_body=vehicle_body, vehicle_config=vehicle_config)
        motion_predictor = StanleyController(control_params=controller_params, vehicle_body=vehicle_body, vehicle_config=vehicle_config)
        super().__init__(
            vehicle_id=vehicle_id,
            vehicle_body=vehicle_body,
            vehicle_config=vehicle_config,
            controller=controller,
            motion_predictor=motion_predictor,
            inst_centric_generator=None,
            intent_predictor=None,
        )
        self.qwen_client = QwenPolicyClient(endpoint=qwen_endpoint, model=qwen_model, timeout=qwen_timeout)
        self.vla_shield = VLASafetyShield()
        self.decision_period = float(decision_period)
        self.max_candidate_spots = int(max_candidate_spots)
        self.entrance_coords = np.asarray(entrance_coords if entrance_coords is not None else [14.38, 76.21], dtype=float)
        self.fallback_spot_index = fallback_spot_index
        self.periodic_replan = bool(periodic_replan)
        self.decision_log_path = decision_log_path
        self._last_vla_decision_time = float("-inf")
        self._inside_vla_apply = False

    def execute_next_task(self):
        if len(self.task_profile) > 0 or self._inside_vla_apply:
            return super().execute_next_task()
        if self.current_task is None:
            if self._make_and_apply_vla_decision(time_value=0.0, reason="initial task selection"):
                return
        return super().execute_next_task()

    def solve(self, time=None):
        time_value = float(time if time is not None else 0.0)
        if self._should_replan(time_value):
            self._make_and_apply_vla_decision(time_value=time_value, reason="scheduled high-level decision")
        return super().solve(time=time)

    def _should_replan(self, time_value: float) -> bool:
        if self.current_task is None:
            return True
        if self.current_task == "END":
            return False
        if self.current_task == "IDLE" and time_value - self._last_vla_decision_time >= self.decision_period:
            return True
        if self.periodic_replan and self.current_task == "CRUISE" and time_value - self._last_vla_decision_time >= self.decision_period:
            return True
        return False

    def _make_and_apply_vla_decision(self, time_value: float, reason: str) -> bool:
        actions = build_candidate_actions(
            self,
            max_spots=self.max_candidate_spots,
            exit_coords=self.entrance_coords,
        )
        if not actions:
            return False
        bev_path = ""
        if self.decision_log_path:
            bev_path = str(Path(self.decision_log_path).with_suffix(".bev.png"))
            try:
                bev_path = save_bev_png(self, bev_path)
            except Exception:
                bev_path = ""
        state = build_vla_state(self, valid_actions=actions, max_spots=max(self.max_candidate_spots, 8))
        context = VLAContext(
            instruction=(
                "Safely complete the parking-lot task. Choose one valid high-level action. "
                "Do not output low-level control. Prefer safe parking progress over unnecessary waiting."
            ),
            state=state,
            valid_actions=actions,
            bev_image_path=bev_path or None,
        )
        decision_started = time.time()
        decision = self.qwen_client.decide(context)
        latency_seconds = time.time() - decision_started
        ok, action, shield_reason = self.vla_shield.validate(decision, actions, vehicle=self)
        if not ok or action is None:
            fallback = choose_default_action(actions)
            if fallback is None:
                self._log_decision(time_value, reason, context, decision, shield_reason, None, latency_seconds)
                return False
            action = fallback
            decision = VLADecision(action_id=action.action_id, reason="fallback after shield rejection: " + shield_reason, used_fallback=True)
        self._inside_vla_apply = True
        try:
            apply_candidate_action(self, action)
        finally:
            self._inside_vla_apply = False
        self._last_vla_decision_time = time_value
        self._log_decision(time_value, reason, context, decision, shield_reason, action, latency_seconds)
        return True

    def _log_decision(self, time_value: float, trigger_reason: str, context: VLAContext, decision: VLADecision, shield_reason: str, action: Any, latency_seconds: float = 0.0) -> None:
        if not self.decision_log_path:
            return
        record = {
            "time": time_value,
            "trigger_reason": trigger_reason,
            "decision": decision.to_dict(),
            "shield_reason": shield_reason,
            "latency_seconds": float(latency_seconds),
            "applied_action": action.to_dict() if action is not None else None,
            "context": context.to_dict(),
        }
        path = Path(self.decision_log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a") as f:
            f.write(json.dumps(record) + "\n")
