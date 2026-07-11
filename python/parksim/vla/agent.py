import json
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np

from parksim.agents.rule_based_stanley_vehicle import RuleBasedStanleyVehicle
from parksim.controller.stanley_controller import StanleyController
from parksim.controller_types import StanleyParams
from parksim.vehicle_types import VehicleBody, VehicleConfig, VehicleTask
from parksim.vla.action_space import apply_candidate_action, build_candidate_actions, choose_default_action
from parksim.vla.baselines import make_baseline_decision
from parksim.vla.bev_encoder import save_bev_png
from parksim.vla.decision_protocol import build_decision_packet
from parksim.vla.fleet_client import QwenFleetPolicyClient
from parksim.vla.fleet_protocol import build_fleet_decision_packet
from parksim.vla.fleet_schema import fleet_context_for_single_vehicle
from parksim.vla.qwen_client import QwenPolicyClient
from parksim.vla.schema import VLAContext, VLADecision
from parksim.vla.shield import VLASafetyShield
from parksim.vla.state_encoder import build_vla_state
from parksim.vla.spot_status import occupancy_ready


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
        reveal_background_intents_to_vla: bool = False,
        fleet_coordinator_enabled: bool = False,
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
        self.qwen_fleet_client = QwenFleetPolicyClient(endpoint=qwen_endpoint, model=qwen_model, timeout=qwen_timeout)
        self.vla_shield = VLASafetyShield()
        self.decision_period = float(decision_period)
        self.max_candidate_spots = int(max_candidate_spots)
        self.entrance_coords = np.asarray(entrance_coords if entrance_coords is not None else [14.38, 76.21], dtype=float)
        self.fallback_spot_index = fallback_spot_index
        self.periodic_replan = bool(periodic_replan)
        self.decision_log_path = decision_log_path
        self.reveal_background_intents_to_vla = bool(reveal_background_intents_to_vla)
        self._last_vla_decision_time = float("-inf")
        self._inside_vla_apply = False
        self._latest_solve_time = 0.0
        self._last_fleet_context = None
        self._last_fleet_response = None
        self.fleet_coordinator_enabled = bool(fleet_coordinator_enabled)
        self._external_fleet_epoch = None
        self._external_fleet_context = None
        self._external_fleet_actions = []
        self._last_fleet_epoch = None
        self._fleet_defer_reason = ""

    def execute_next_task(self):
        if self.fleet_coordinator_enabled and not self._inside_vla_apply:
            if len(self.task_profile) == 0 and self.current_task in (None, "IDLE"):
                self._hold_for_world_state()
                return
            return super().execute_next_task()
        if len(self.task_profile) > 0 or self._inside_vla_apply:
            return super().execute_next_task()
        if self.current_task in (None, "IDLE"):
            if not occupancy_ready(self):
                self._hold_for_world_state()
                return
            decision_time = float(self._latest_solve_time)
            if self._make_and_apply_vla_decision(time_value=decision_time, reason="initial task selection"):
                return
        return super().execute_next_task()

    def solve(self, time=None):
        time_value = float(time if time is not None else 0.0)
        self._latest_solve_time = time_value
        if self.fleet_coordinator_enabled:
            if len(self.task_profile) == 0 and self.current_task in (None, "IDLE"):
                self._hold_for_world_state()
            return super().solve(time=time)
        if self._should_replan(time_value):
            if occupancy_ready(self):
                self._make_and_apply_vla_decision(time_value=time_value, reason="scheduled high-level decision")
            else:
                self._hold_for_world_state()
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

    def _hold_for_world_state(self) -> None:
        if self.current_task == "IDLE" and self.idle_duration is not None:
            return
        self._inside_vla_apply = True
        try:
            self.set_task_profile([VehicleTask(name="IDLE", duration=0.2)])
            super().execute_next_task()
        finally:
            self._inside_vla_apply = False

    def build_fleet_epoch_context(self, epoch_id: int, sim_time: float) -> Optional[VLAContext]:
        """Build one immutable high-level decision context for a central epoch."""
        self._fleet_defer_reason = ""
        if not self.fleet_coordinator_enabled or self.is_all_done():
            self._fleet_defer_reason = "inactive_or_done"
            return None
        # A cloud epoch must never ask Qwen to choose from an all-unknown lot.
        # Returning None causes the ROS coordinator to defer this AV and retry
        # at a later simulation-aligned epoch after occupancy has propagated.
        if not occupancy_ready(self):
            self._fleet_defer_reason = "occupancy_not_ready"
            return None
        if self._has_active_low_level_maneuver() and not self._active_maneuver_replan_due(sim_time):
            self._fleet_defer_reason = "active_low_level_maneuver"
            return None
        actions = build_candidate_actions(
            self,
            max_spots=self.max_candidate_spots,
            exit_coords=self.entrance_coords,
        )
        if not actions:
            self._fleet_defer_reason = "no_candidate_actions"
            return None
        state = build_vla_state(self, valid_actions=actions, max_spots=max(self.max_candidate_spots, 8))
        context = VLAContext(
            instruction=(
                "Choose one high-level action from valid_actions for this automated vehicle. "
                "The cloud coordinator combines this context with every other AV before "
                "making a fleet-level reservation, priority, and route-bundle decision. "
                "Occupied, unknown, blocked, and reserved parking spots are never selectable. "
                "Human/replay vehicle intent is partially observable; use uncertainty and "
                "observable motion only. Do not output low-level control."
            ),
            state=state,
            valid_actions=actions,
        )
        self._external_fleet_epoch = int(epoch_id)
        self._external_fleet_context = context
        self._external_fleet_actions = actions
        self._last_fleet_context = None
        self._last_fleet_response = None
        return context

    def apply_fleet_epoch_decision(self, epoch_id: int, payload: dict, sim_time: float) -> bool:
        """Apply the coordinator-approved bundle without a vehicle HTTP call."""
        if not self.fleet_coordinator_enabled:
            return False
        if int(epoch_id) != int(self._external_fleet_epoch or -1):
            return False
        actions = list(self._external_fleet_actions or [])
        decision = VLADecision(
            action_id=str(payload.get("action_id", "")),
            target_spot_index=payload.get("target_spot_index"),
            reason_code=str(payload.get("reason_code", "")),
            reason=str(payload.get("reason", "")),
            confidence=float(payload.get("confidence", 0.0) or 0.0),
            raw_response=json.dumps(payload.get("raw_response", "")),
            used_fallback=bool(payload.get("used_fallback", False)),
        )
        ok, action, shield_reason = self.vla_shield.validate(decision, actions, vehicle=self)
        if not ok or action is None:
            action = choose_default_action(actions)
            if action is None:
                return False
            decision = VLADecision(
                action_id=action.action_id,
                target_spot_index=action.target_spot_index,
                reason="single-vehicle fallback after fleet shield rejection: " + shield_reason,
                reason_code="FALLBACK_OR_RECOVERY",
                used_fallback=True,
            )
        self.priority = int(payload.get("priority", getattr(self, "priority", 0)) or 0)
        applied_shield_reason = str(payload.get("shield_reason", shield_reason))
        self._inside_vla_apply = True
        try:
            apply_candidate_action(self, action)
        finally:
            self._inside_vla_apply = False
        recovery_reason = str(getattr(self, "_vla_execution_recovery_reason", "") or "")
        if recovery_reason:
            recovery_action = next((candidate for candidate in actions if candidate.action_id.startswith("wait_")), None)
            if recovery_action is not None:
                action = recovery_action
                decision = VLADecision(
                    action_id=action.action_id,
                    target_spot_index=None,
                    reason="fallback after executor path recovery",
                    reason_code="EXECUTOR_PATH_RECOVERY",
                    used_fallback=True,
                )
            applied_shield_reason = recovery_reason
        self._last_vla_decision_time = float(sim_time)
        self._last_fleet_epoch = int(epoch_id)
        self._log_decision(
            sim_time,
            "synchronized fleet epoch %d" % int(epoch_id),
            self._external_fleet_context,
            decision,
            applied_shield_reason,
            action,
            0.0,
        )
        return True

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
                "Safely complete the parking-lot task by choosing exactly one action_id from valid_actions. "
                "Each valid action is a fixed spot-route-wait bundle; compare bundle_cost, reservation_cost, centralized_assignment_cost, conflict_risk, expected_wait_s, and path length. "
                "Spots with status occupied or unknown are not selectable and must not be chosen. "
                "candidate_spots contains only verified available spots; blocked_nearby_spots is explanatory only. "
                "Background human/rule vehicle intent is partially observable, so use pose, speed, braking, and uncertainty fields. "
                "Do not output low-level control. Prefer lower-cost safe parking progress over unnecessary waiting."
            ),
            state=state,
            valid_actions=actions,
            bev_image_path=bev_path or None,
        )
        self._last_fleet_context = None
        self._last_fleet_response = None
        decision_started = time.time()
        decision = self._decide(context)
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

    def _decide(self, context: VLAContext) -> VLADecision:
        fleet_context = fleet_context_for_single_vehicle(context)
        fleet_response = self.qwen_fleet_client.decide_fleet(fleet_context)
        self._last_fleet_context = fleet_context
        self._last_fleet_response = fleet_response
        state = context.state if isinstance(context.state, dict) else {}
        ego = state.get("ego", {}) if isinstance(state, dict) else {}
        vehicle_id = int(ego.get("vehicle_id", self.vehicle_id))
        fleet_decision = fleet_response.decision_for(vehicle_id)
        if fleet_decision is None:
            fallback = self.qwen_client.decide(context)
            fallback.used_fallback = True
            fallback.reason = "fleet response missing this vehicle; " + fallback.reason
            return fallback
        return fleet_decision.to_vla_decision()

    def _log_decision(self, time_value: float, trigger_reason: str, context: VLAContext, decision: VLADecision, shield_reason: str, action: Any, latency_seconds: float = 0.0) -> None:
        if not self.decision_log_path:
            return
        decision_packet = build_decision_packet(context)
        fleet_packet = None
        fleet_response = None
        if self._last_fleet_context is not None:
            fleet_packet = build_fleet_decision_packet(self._last_fleet_context)
        if self._last_fleet_response is not None:
            fleet_response = self._last_fleet_response.to_dict()
        record = {
            "time": time_value,
            "sim_time": float(time_value),
            "trigger_reason": trigger_reason,
            "cloud_decision_scope": "fleet_level_qwen_vla_server",
            "fleet_vehicle_id": int(self.vehicle_id),
            "fleet_epoch": self._last_fleet_epoch,
            "fleet_decision_mode": "synchronized_fleet_epoch" if self.fleet_coordinator_enabled else "legacy_single_vehicle_trigger",
            "protocol_version": decision_packet.get("protocol_version"),
            "prompt_version": decision_packet.get("prompt_version"),
            "fleet_protocol_version": fleet_packet.get("protocol_version") if fleet_packet else None,
            "fleet_prompt_version": fleet_packet.get("prompt_version") if fleet_packet else None,
            "valid_action_ids": decision_packet.get("valid_action_ids", []),
            "decision": decision.to_dict(),
            "shield_reason": shield_reason,
            "latency_seconds": float(latency_seconds),
            "applied_action": action.to_dict() if action is not None else None,
            "decision_packet": decision_packet,
            "fleet_decision_packet": fleet_packet,
            "fleet_response": fleet_response,
            "context": context.to_dict(),
        }
        path = Path(self.decision_log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("a") as f:
            f.write(json.dumps(record) + "\n")

    def _has_active_low_level_maneuver(self) -> bool:
        """Cloud policy may choose tasks only at stable task boundaries."""
        return str(self.current_task or "").upper() in ("UNPARK", "CRUISE", "PARK")

    def _active_maneuver_replan_due(self, sim_time: float) -> bool:
        """Allow recovery only for a stalled/yielding active maneuver.

        A fleet epoch must not overwrite a moving CRUISE/PARK/UNPARK executor.
        Doing so can interrupt ParkSim's offline parking maneuver and expose a
        second spot assignment for an AV that is already committed to a bay.
        """
        last_decision = float(self._last_vla_decision_time)
        is_yielding = bool(getattr(self, "is_braking", False)) or int(getattr(self, "waiting_for", 0) or 0) != 0
        if not np.isfinite(last_decision):
            return is_yielding
        if float(sim_time) - last_decision < self.decision_period:
            return False
        if is_yielding:
            return True
        try:
            speed = abs(float(self.state.v.v))
        except Exception:
            speed = 0.0
        return speed < 0.15

class BaselineVLAVehicle(QwenVLAVehicle):
    """Deterministic high-level VLA baseline over the same action interface."""

    def __init__(self, *args, baseline_strategy: str = "risk_aware_rule", **kwargs):
        self.baseline_strategy = str(baseline_strategy or "risk_aware_rule").lower()
        if self.baseline_strategy == "oracle_intent_bundle":
            kwargs["reveal_background_intents_to_vla"] = True
        kwargs.setdefault("qwen_endpoint", "")
        kwargs.setdefault("qwen_model", "baseline:%s" % self.baseline_strategy)
        kwargs.setdefault("qwen_timeout", 0.0)
        super().__init__(*args, **kwargs)
        self.baseline_strategy = str(baseline_strategy or "risk_aware_rule").lower()

    def _decide(self, context: VLAContext) -> VLADecision:
        return make_baseline_decision(context.valid_actions, self.baseline_strategy)
