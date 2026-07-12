"""Synchronized cloud-fleet decision epochs independent of ROS transport."""
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import time

from parksim.vla.fleet_bev import save_fleet_bev_png
from parksim.vla.fleet_client import QwenFleetPolicyClient
from parksim.vla.fleet_critic import FleetDecisionCritic
from parksim.vla.fleet_schema import VLAFleetContext
from parksim.vla.fleet_shield import VLAFleetSafetyShield
from parksim.vla.intent_belief import HumanIntentBeliefTracker
from parksim.vla.schema import VLACandidateAction, VLAContext


def _as_int(value: Any, default: Optional[int] = None) -> Optional[int]:
    try:
        return int(value)
    except (TypeError, ValueError):
        return default


def context_from_packet(packet: Dict[str, Any]) -> VLAContext:
    row = packet.get("context", packet)
    if not isinstance(row, dict):
        raise ValueError("fleet context must be a JSON object")
    actions: List[VLACandidateAction] = []
    for item in row.get("valid_actions", []) or []:
        if not isinstance(item, dict):
            continue
        actions.append(VLACandidateAction(
            action_id=str(item.get("action_id", "")),
            action_type=str(item.get("action_type", "")),
            target_spot_index=_as_int(item.get("target_spot_index")),
            target_coords=item.get("target_coords"),
            duration=item.get("duration"),
            route_id=item.get("route_id"),
            reason=str(item.get("reason", "")),
            features=dict(item.get("features") or {}),
        ))
    return VLAContext(
        instruction=str(row.get("instruction", "")),
        state=dict(row.get("state") or {}),
        valid_actions=actions,
        bev_image_path=row.get("bev_image_path"),
        protocol_version=str(row.get("protocol_version") or "ParkSim-Qwen-VLA-Decision-v1"),
        prompt_version=str(row.get("prompt_version") or "qwen-vla-high-level-policy-v1"),
    )


@dataclass
class FleetEpoch:
    epoch_id: int
    sim_time: float
    expected_vehicle_ids: List[int]
    started_wall_time: float
    contexts: Dict[int, VLAContext] = field(default_factory=dict)
    deferred_vehicle_ids: Set[int] = field(default_factory=set)
    deferred_reasons: Dict[int, str] = field(default_factory=dict)


class FleetEpochCoordinator:
    """Collect one AV context per epoch, then call and validate one fleet policy."""

    def __init__(
        self,
        decision_period: float = 3.0,
        bev_output_dir: str = "",
        occupancy_retry_interval: float = 1.0,
    ) -> None:
        self.decision_period = max(0.1, float(decision_period))
        self.occupancy_retry_interval = max(0.1, float(occupancy_retry_interval))
        self.bev_output_dir = str(bev_output_dir or "")
        self.registered_vehicle_ids: Set[int] = set()
        self.next_epoch_sim_time = 0.0
        self.epoch_counter = 0
        self.active_epoch: Optional[FleetEpoch] = None
        self.critic = FleetDecisionCritic()
        self.human_belief_tracker = HumanIntentBeliefTracker()
        self.previous_outcome_summary: Dict[str, Any] = {}

    def register(self, vehicle_id: int) -> None:
        self.registered_vehicle_ids.add(int(vehicle_id))

    def unregister(self, vehicle_id: int) -> None:
        vehicle_id = int(vehicle_id)
        self.registered_vehicle_ids.discard(vehicle_id)
        if self.active_epoch is not None:
            self.active_epoch.expected_vehicle_ids = [
                item for item in self.active_epoch.expected_vehicle_ids if item != vehicle_id
            ]
            self.active_epoch.contexts.pop(vehicle_id, None)
            self.active_epoch.deferred_reasons.pop(vehicle_id, None)

    def should_start(self, sim_time: float) -> bool:
        return (
            self.active_epoch is None
            and bool(self.registered_vehicle_ids)
            and float(sim_time) + 1e-9 >= self.next_epoch_sim_time
        )

    def start(self, sim_time: float, wall_time: float) -> Optional[FleetEpoch]:
        if not self.should_start(sim_time):
            return None
        self.epoch_counter += 1
        self.active_epoch = FleetEpoch(
            epoch_id=self.epoch_counter,
            sim_time=float(sim_time),
            expected_vehicle_ids=sorted(self.registered_vehicle_ids),
            started_wall_time=float(wall_time),
        )
        self.next_epoch_sim_time = float(sim_time) + self.decision_period
        return self.active_epoch

    def accept_context(self, packet: Dict[str, Any]) -> bool:
        epoch = self.active_epoch
        if epoch is None:
            return False
        epoch_id = _as_int(packet.get("epoch_id"))
        vehicle_id = _as_int(packet.get("vehicle_id"))
        if epoch_id != epoch.epoch_id or vehicle_id not in epoch.expected_vehicle_ids:
            return False
        if not bool(packet.get("ready", True)):
            return False
        context = context_from_packet(packet)
        ego = context.state.get("ego", {}) if isinstance(context.state, dict) else {}
        if _as_int(ego.get("vehicle_id")) != vehicle_id or not context.valid_actions:
            return False
        epoch.contexts[vehicle_id] = context
        return True

    def defer_context(self, packet: Dict[str, Any]) -> bool:
        """Exclude a vehicle from this epoch and retain its auditable defer reason."""
        epoch = self.active_epoch
        if epoch is None:
            return False
        epoch_id = _as_int(packet.get("epoch_id"))
        vehicle_id = _as_int(packet.get("vehicle_id"))
        if epoch_id != epoch.epoch_id or vehicle_id not in epoch.expected_vehicle_ids:
            return False
        epoch.expected_vehicle_ids = [item for item in epoch.expected_vehicle_ids if item != vehicle_id]
        epoch.contexts.pop(vehicle_id, None)
        epoch.deferred_vehicle_ids.add(int(vehicle_id))
        epoch.deferred_reasons[int(vehicle_id)] = str(packet.get("defer_reason", "not_ready") or "not_ready")
        return True

    def is_complete(self) -> bool:
        epoch = self.active_epoch
        return epoch is not None and set(epoch.expected_vehicle_ids).issubset(epoch.contexts)

    def timed_out(self, wall_time: float, timeout: float) -> bool:
        epoch = self.active_epoch
        return epoch is not None and float(wall_time) - epoch.started_wall_time >= max(0.0, float(timeout))

    def finalize(
        self,
        client: QwenFleetPolicyClient,
        shield: Optional[VLAFleetSafetyShield] = None,
        run_id: str = "",
        policy_mode: str = "direct",
    ) -> Dict[str, Any]:
        epoch = self.active_epoch
        if epoch is None:
            raise RuntimeError("no active fleet epoch")
        shield = shield or VLAFleetSafetyShield()
        collected_ids = sorted(epoch.contexts)
        missing_ids = [item for item in epoch.expected_vehicle_ids if item not in epoch.contexts]
        contexts = [epoch.contexts[item] for item in collected_ids]
        fleet_bev_path = None
        if contexts and self.bev_output_dir:
            try:
                fleet_bev_path = save_fleet_bev_png(
                    contexts,
                    str(Path(self.bev_output_dir) / ("fleet_epoch_%05d.png" % epoch.epoch_id)),
                )
            except Exception:
                fleet_bev_path = None
        fleet_state = self._fleet_state(epoch, run_id, missing_ids)
        fleet_context = VLAFleetContext(
            instruction=(
                "Act as the synchronized cloud multimodal-LLM coordinator for the entire mixed "
                "human-autonomous parking lot. Return one executable high-level action "
                "for every automated vehicle in this decision epoch."
            ),
            state=fleet_state,
            vehicle_contexts=contexts,
            bev_image_path=fleet_bev_path,
        )
        conflict_graph = self.critic.candidate_conflict_graph(fleet_context)
        fleet_context.state["candidate_conflict_graph"] = conflict_graph
        result: Dict[str, Any] = {
            "run_id": str(run_id),
            "epoch_id": epoch.epoch_id,
            "sim_time": epoch.sim_time,
            "expected_vehicle_ids": list(epoch.expected_vehicle_ids),
            "collected_vehicle_ids": collected_ids,
            "missing_vehicle_ids": missing_ids,
            "deferred_vehicle_ids": sorted(epoch.deferred_vehicle_ids),
            "deferred_reasons": {str(key): value for key, value in sorted(epoch.deferred_reasons.items())},
            "decision_scope": "synchronized_fleet_epoch",
            "decision_complete": not missing_ids,
            "fleet_context": fleet_context.to_dict(),
            "fleet_bev_path": fleet_bev_path,
            "fleet_decisions": [],
            "policy_mode": str(policy_mode),
            "model_id": (
                "deterministic_fleet_min_cost"
                if str(policy_mode).lower() == "fleet_min_cost"
                else str(getattr(client, "model", "unknown"))
            ),
            "model_revision": "not_applicable" if str(policy_mode).lower() == "fleet_min_cost" else "local_snapshot",
            "feedback_schema_version": "ParkSim-MLLM-Fleet-Critic-v1",
            "repair_attempted": False,
            "repair_latency_seconds": 0.0,
        }
        if collected_ids:
            normalized_mode = str(policy_mode or "direct").lower()
            if normalized_mode == "fleet_min_cost":
                initial_response = self.critic.optimize(fleet_context)
            else:
                initial_response = client.decide_fleet(fleet_context)
            pre_critique = self.critic.evaluate(initial_response, fleet_context, conflict_graph)
            response = initial_response
            if normalized_mode == "external_feedback" and pre_critique["needs_repair"]:
                repair_started = time.monotonic()
                response = client.repair_fleet(fleet_context, initial_response, pre_critique, mode="external_feedback")
                result["repair_latency_seconds"] = float(time.monotonic() - repair_started)
                result["repair_attempted"] = True
            elif normalized_mode == "self_reflect":
                repair_started = time.monotonic()
                response = client.repair_fleet(fleet_context, initial_response, mode="self_reflect")
                result["repair_latency_seconds"] = float(time.monotonic() - repair_started)
                result["repair_attempted"] = True
            post_critique = self.critic.evaluate(response, fleet_context, conflict_graph)
            checked = shield.validate(response, fleet_context)
            result["initial_fleet_response"] = initial_response.to_dict()
            result["raw_fleet_response"] = response.to_dict()
            result["pre_feedback_critique"] = pre_critique
            result["post_feedback_critique"] = post_critique
            result["changed_by_feedback_count"] = _changed_decision_count(initial_response, response)
            pre_issue_count = int(pre_critique["hard_violation_count"]) + int(pre_critique["quality_warning_count"])
            post_issue_count = int(post_critique["hard_violation_count"]) + int(post_critique["quality_warning_count"])
            result["repair_success"] = bool(result["repair_attempted"] and post_issue_count < pre_issue_count)
            for vehicle_id in collected_ids:
                ok, action, reason, decision = checked[vehicle_id]
                result["fleet_decisions"].append({
                    **decision.to_dict(),
                    "vehicle_id": int(vehicle_id),
                    "shield_ok": bool(ok),
                    "shield_reason": str(reason),
                    "action_bundle": action.to_dict() if action is not None else None,
                })
            self.previous_outcome_summary = {
                "epoch_id": int(epoch.epoch_id),
                "post_feedback_objective_score": post_critique.get("objective_score"),
                "post_feedback_hard_violation_count": post_critique.get("hard_violation_count"),
                "shield_rejection_count": sum(1 for item in result["fleet_decisions"] if not item["shield_ok"]),
                "executed_action_ids": {str(item["vehicle_id"]): item.get("action_id") for item in result["fleet_decisions"]},
            }
        else:
            result["raw_fleet_response"] = {"fleet_decisions": [], "raw_response": ""}
        if not collected_ids and any(reason == "occupancy_not_ready" for reason in epoch.deferred_reasons.values()):
            self.next_epoch_sim_time = min(
                self.next_epoch_sim_time,
                float(epoch.sim_time) + self.occupancy_retry_interval,
            )
        self.active_epoch = None
        return result

    def _fleet_state(self, epoch: FleetEpoch, run_id: str, missing_ids: List[int]) -> Dict[str, Any]:
        humans: Dict[int, Dict[str, Any]] = {}
        av_states: List[Dict[str, Any]] = []
        for vehicle_id in sorted(epoch.contexts):
            state = epoch.contexts[vehicle_id].state
            ego = dict(state.get("ego") or {})
            ego["vehicle_id"] = int(vehicle_id)
            av_states.append(ego)
            for row in state.get("nearby_vehicles", []) or []:
                if not isinstance(row, dict):
                    continue
                other_id = _as_int(row.get("vehicle_id"))
                if other_id is not None and other_id not in self.registered_vehicle_ids:
                    humans[other_id] = dict(row)
        beliefs = self.human_belief_tracker.update(epoch.sim_time, humans.values())
        return {
            "cloud_policy_role": "fleet_level_multimodal_llm_server",
            "decision_scope": "synchronized_fleet_epoch",
            "run_id": str(run_id),
            "sim_time": float(epoch.sim_time),
            "epoch_id": int(epoch.epoch_id),
            "registered_av_ids": list(epoch.expected_vehicle_ids),
            "context_received_av_ids": sorted(epoch.contexts),
            "context_missing_av_ids": list(missing_ids),
            "deferred_vehicle_ids": sorted(epoch.deferred_vehicle_ids),
            "automated_vehicle_states": av_states,
            "observable_human_vehicle_states": list(humans.values()),
            "human_intent_beliefs": beliefs,
            "human_intent_model": "partially_observable_replay_rule_random_mixed",
            "human_information_boundary": {
                "operation_class_observable": True,
                "target_spot_observable": False,
                "intended_route_observable": False,
                "future_ground_truth_trajectory_observable": False,
                "belief_trajectories_are_inferred_not_privileged": True,
            },
            "previous_outcome_summary": dict(self.previous_outcome_summary),
            "sim_time_alignment": "cloud latency is logged while simulator advancement is paused",
        }


def _changed_decision_count(initial: Any, revised: Any) -> int:
    initial_by_id = {int(item.vehicle_id): item for item in initial.decisions}
    revised_by_id = {int(item.vehicle_id): item for item in revised.decisions}
    vehicle_ids = set(initial_by_id) | set(revised_by_id)
    return sum(
        1 for vehicle_id in vehicle_ids
        if vehicle_id not in initial_by_id
        or vehicle_id not in revised_by_id
        or initial_by_id[vehicle_id].action_id != revised_by_id[vehicle_id].action_id
    )
