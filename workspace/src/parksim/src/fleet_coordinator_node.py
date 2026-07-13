#!/usr/bin/env python3
"""ROS transport for synchronized ParkSim cloud-fleet VLA epochs."""
import json
import os
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from parksim.vla.fleet_client import QwenFleetPolicyClient
from parksim.vla.fleet_coordinator import FleetEpochCoordinator
from parksim.vla.fleet_shield import VLAFleetSafetyShield
from parksim.vla.sync_barrier import (
    barrier_audit_fields,
    decision_barrier_status,
    validate_decision_ack,
    validate_pause_ack,
)


class FleetCoordinatorNode(Node):
    def __init__(self):
        super().__init__("fleet_coordinator")
        self.declare_parameter("qwen_endpoint", "")
        self.declare_parameter("qwen_model", "Qwen2.5-VL-7B-Instruct")
        self.declare_parameter("qwen_timeout", 60.0)
        self.declare_parameter("decision_period", 3.0)
        self.declare_parameter("batch_timeout", 5.0)
        self.declare_parameter("run_id", "")
        self.declare_parameter("decision_log_path", "")
        self.declare_parameter("policy_mode", "direct")
        self.declare_parameter("decision_ack_timeout", 10.0)

        self.run_id = str(self.get_parameter("run_id").value)
        self.batch_timeout = float(self.get_parameter("batch_timeout").value)
        self.policy_mode = str(self.get_parameter("policy_mode").value or "direct")
        self.decision_ack_timeout = max(0.1, float(self.get_parameter("decision_ack_timeout").value))
        self.client = QwenFleetPolicyClient(
            endpoint=str(self.get_parameter("qwen_endpoint").value),
            model=str(self.get_parameter("qwen_model").value),
            timeout=float(self.get_parameter("qwen_timeout").value),
        )
        self.decision_log_path = str(self.get_parameter("decision_log_path").value) or "fleet_epochs.jsonl"
        self.coordinator = FleetEpochCoordinator(
            decision_period=float(self.get_parameter("decision_period").value),
            bev_output_dir=os.path.join(os.path.dirname(self.decision_log_path) or ".", "fleet_bev"),
        )
        self.shield = VLAFleetSafetyShield()
        self.sim_time = None
        self.pause_requested = False
        self.pause_requested_sim_time = None
        self.pause_ack_payload = None
        self.pending_decision_payload = None
        self.decision_ack_payloads = {}
        self.decision_published_wall_time = None
        self.barrier_terminal_failure = False

        self.pause_pub = self.create_publisher(Bool, "/vla/fleet_pause", 10)
        self.epoch_pub = self.create_publisher(String, "/vla/fleet_epoch", 10)
        self.decisions_pub = self.create_publisher(String, "/vla/fleet_decisions", 10)
        self.create_subscription(Float32, "/sim_time", self._sim_time_cb, 10)
        self.create_subscription(String, "/vla/fleet_registry", self._registry_cb, 10)
        self.create_subscription(String, "/vla/fleet_context", self._context_cb, 10)
        self.create_subscription(String, "/vla/fleet_pause_ack", self._pause_ack_cb, 10)
        self.create_subscription(String, "/vla/fleet_decision_ack", self._decision_ack_cb, 10)
        self.timer = self.create_timer(0.05, self._timer_cb)
        self.get_logger().info("Fleet coordinator initialized run_id=%s" % self.run_id)

    def _same_run(self, payload):
        packet_run_id = str(payload.get("run_id", ""))
        return not packet_run_id or not self.run_id or packet_run_id == self.run_id

    def _sim_time_cb(self, msg):
        self.sim_time = float(msg.data)

    def _registry_cb(self, msg):
        try:
            payload = json.loads(msg.data)
            vehicle_id = int(payload.get("vehicle_id"))
        except (TypeError, ValueError):
            return
        if not isinstance(payload, dict) or not self._same_run(payload):
            return
        if str(payload.get("event", "register")) == "unregister":
            self.coordinator.unregister(vehicle_id)
        else:
            self.coordinator.register(vehicle_id)

    def _context_cb(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        if isinstance(payload, dict) and self._same_run(payload):
            if bool(payload.get("ready", True)):
                self.coordinator.accept_context(payload)
            else:
                self.coordinator.defer_context(payload)

    def _publish_pause(self, paused):
        message = Bool()
        message.data = bool(paused)
        self.pause_pub.publish(message)

    def _pause_ack_cb(self, msg):
        if not self.pause_requested or self.coordinator.active_epoch is not None:
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        self.pause_ack_payload = validate_pause_ack(
            payload,
            self.run_id,
            float(self.pause_requested_sim_time or 0.0),
            epoch_active=self.coordinator.active_epoch is not None,
        )

    def _decision_ack_cb(self, msg):
        pending = self.pending_decision_payload
        if pending is None:
            return
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return
        accepted = validate_decision_ack(payload, self.run_id, pending)
        if accepted is not None:
            self.decision_ack_payloads[accepted[0]] = accepted[1]

    def _finish_decision_barrier(self, now):
        payload = self.pending_decision_payload
        payload.update(barrier_audit_fields(
            payload,
            self.decision_ack_payloads,
            now,
            self.decision_published_wall_time,
            "applied",
        ))
        self._append_log(payload)
        self._publish_pause(False)
        self.pause_requested = False
        self.pause_requested_sim_time = None
        self.pause_ack_payload = None
        self.pending_decision_payload = None
        self.decision_ack_payloads = {}
        self.decision_published_wall_time = None

    def _fail_decision_barrier(self, now, status):
        payload = self.pending_decision_payload
        payload.update(barrier_audit_fields(
            payload,
            self.decision_ack_payloads,
            now,
            self.decision_published_wall_time,
            status,
        ))
        self._append_log(payload)
        self.barrier_terminal_failure = True
        self.get_logger().error("Fleet decision barrier failed: %s; simulator remains paused" % status)

    def _append_log(self, payload):
        directory = os.path.dirname(self.decision_log_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.decision_log_path, "a") as handle:
            handle.write(json.dumps(payload) + "\n")

    def _timer_cb(self):
        if self.sim_time is None:
            return
        now = time.monotonic()
        if self.barrier_terminal_failure:
            return
        if self.pending_decision_payload is not None:
            status = decision_barrier_status(
                self.pending_decision_payload,
                self.decision_ack_payloads,
                now,
                self.decision_published_wall_time,
                self.decision_ack_timeout,
            )
            if status == "applied":
                self._finish_decision_barrier(now)
            elif status not in ("pending", "idle"):
                self._fail_decision_barrier(now, status)
            return
        active = self.coordinator.active_epoch
        if active is None:
            if not self.coordinator.should_start(self.sim_time):
                return
            if not self.pause_requested:
                self.pause_requested = True
                self.pause_requested_sim_time = float(self.sim_time)
                self.pause_ack_payload = None
                self._publish_pause(True)
                return
            if self.pause_ack_payload is None:
                return
            epoch = self.coordinator.start(float(self.pause_ack_payload["sim_time"]), now)
            if epoch is None:
                return
            message = String()
            message.data = json.dumps({
                "run_id": self.run_id,
                "epoch_id": epoch.epoch_id,
                "sim_time": epoch.sim_time,
                "expected_vehicle_ids": epoch.expected_vehicle_ids,
            })
            self.epoch_pub.publish(message)
            return
        if not self.coordinator.is_complete() and not self.coordinator.timed_out(now, self.batch_timeout):
            return

        started = time.monotonic()
        payload = self.coordinator.finalize(
            self.client,
            self.shield,
            run_id=self.run_id,
            policy_mode=self.policy_mode,
        )
        payload["latency_seconds"] = float(time.monotonic() - started)
        payload["batch_wait_seconds"] = float(started - active.started_wall_time)
        payload["pause_ack_sim_time"] = float((self.pause_ack_payload or {}).get("sim_time", active.sim_time))
        payload["simulation_time_policy"] = "decision_then_advance"
        payload["wall_clock_latency_in_performance_metrics"] = False
        message = String()
        message.data = json.dumps(payload)
        self.pending_decision_payload = payload
        self.decision_ack_payloads = {}
        self.decision_published_wall_time = time.monotonic()
        self.decisions_pub.publish(message)
        if not payload.get("fleet_decisions"):
            self._finish_decision_barrier(time.monotonic())


def main(args=None):
    rclpy.init(args=args)
    node = FleetCoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
