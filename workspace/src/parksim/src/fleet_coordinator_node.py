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

        self.run_id = str(self.get_parameter("run_id").value)
        self.batch_timeout = float(self.get_parameter("batch_timeout").value)
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

        self.pause_pub = self.create_publisher(Bool, "/vla/fleet_pause", 10)
        self.epoch_pub = self.create_publisher(String, "/vla/fleet_epoch", 10)
        self.decisions_pub = self.create_publisher(String, "/vla/fleet_decisions", 10)
        self.create_subscription(Float32, "/sim_time", self._sim_time_cb, 10)
        self.create_subscription(String, "/vla/fleet_registry", self._registry_cb, 10)
        self.create_subscription(String, "/vla/fleet_context", self._context_cb, 10)
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
            self.coordinator.accept_context(payload)

    def _publish_pause(self, paused):
        message = Bool()
        message.data = bool(paused)
        self.pause_pub.publish(message)

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
        active = self.coordinator.active_epoch
        if active is None:
            epoch = self.coordinator.start(self.sim_time, now)
            if epoch is None:
                return
            self._publish_pause(True)
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
        payload = self.coordinator.finalize(self.client, self.shield, run_id=self.run_id)
        payload["latency_seconds"] = float(time.monotonic() - started)
        payload["batch_wait_seconds"] = float(started - active.started_wall_time)
        self._append_log(payload)
        message = String()
        message.data = json.dumps(payload)
        self.decisions_pub.publish(message)
        self._publish_pause(False)


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

