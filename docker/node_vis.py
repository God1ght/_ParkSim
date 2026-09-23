#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""容器内节点可见性探针（不依赖 ros2cli/netifaces）。

在指定容器里跑，列出该 DDS 环境下能发现的所有 ROS 2 节点。
隔离正常（ROS_LOCALHOST_ONLY=1）时，只应看到本容器自己的
webviz_bridge / simulator / vehicle_*；不应出现任何"外来" vehicle 节点。

用法：python3 node_vis.py [label]
"""
import sys
import time

import rclpy
from rclpy.node import Node

LABEL = sys.argv[1] if len(sys.argv) > 1 else "?"
rclpy.init()
node = Node("vis_probe")
deadline = time.time() + 5.0
seen = {}
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    for name, ns in node.get_node_names_and_namespaces():
        seen[(ns, name)] = True

print("=== [%s] discovered %d node(s) ===" % (LABEL, len(seen)))
for (ns, name) in sorted(seen):
    print("  %s%s" % (ns if ns.endswith("/") else ns + "/", name))
node.destroy_node()
rclpy.shutdown()
