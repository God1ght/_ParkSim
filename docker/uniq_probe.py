#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""唯一命名节点交叉可见性测试（DDS 隔离决定性证据）。

在容器 A 里起一个长生命周期节点 /<label>_UNIQUE_PROBE，
再从容器 B/C 里做节点发现，看是否能看到 A 的探针。
看不到 => DDS 发现被隔离；看得到 => 存在跨容器发现。

用法：
  节点侧： python3 uniq_probe.py hold <label> [seconds]
  观察侧： python3 uniq_probe.py scan <label>
"""
import sys
import time

import rclpy
from rclpy.node import Node

mode = sys.argv[1] if len(sys.argv) > 1 else "scan"
label = sys.argv[2] if len(sys.argv) > 2 else "X"
name = "%s_UNIQUE_PROBE" % label

rclpy.init()

if mode == "hold":
    secs = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0
    node = Node(name)
    print("HOLDING %s for %.0fs" % (name, secs), flush=True)
    end = time.time() + secs
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node()
    rclpy.shutdown()
    print("RELEASED %s" % name, flush=True)
    sys.exit(0)

# scan
node = Node("scanner_%s" % label)
target = None
deadline = time.time() + 8.0
worst = {}
while time.time() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)
    for nm, ns in node.get_node_names_and_namespaces():
        full = "%s%s" % (ns if ns.endswith("/") else ns + "/", nm)
        worst[full] = True
        if nm.endswith("_UNIQUE_PROBE") or "_UNIQUE_PROBE" in full:
            target = full
print("[scan %s] total_discovered=%d" % (label, len(worst)))
probes = sorted(k for k in worst if "UNIQUE_PROBE" in k)
print("[scan %s] probes_visible=%s" % (label, probes if probes else "NONE"))
node.destroy_node()
rclpy.shutdown()
sys.exit(0)
