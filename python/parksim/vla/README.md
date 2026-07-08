# ParkSim VLA Prototype

This package implements a high-level VLA decision layer for ParkSim. The model chooses task-level actions such as waiting, selecting a parking spot, cruising to a spot, rerouting, or parking. Low-level steering and acceleration remain handled by the existing ParkSim rule-based planner and Stanley controller.

The Qwen endpoint is optional for smoke testing. If it is unset or unavailable, the policy falls back to a deterministic valid action so that ROS integration can be tested before a local Qwen-VL service is deployed.
