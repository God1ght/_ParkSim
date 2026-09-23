#!/usr/bin/env bash
# =============================================================================
# DDS 隔离决定性验证（唯一命名节点交叉可见性）
#
# 场景：iso1/iso2 = 新镜像（镜像内固化 ROS_LOCALHOST_ONLY=1）
#       ps       = 8098 demo，旧镜像（ROS_LOCALHOST_ONLY 未设 = 0）→ 故意不隔离
#
# 步骤：
#   A) 在 iso1 起长生命周期节点 /ISO1_UNIQUE_PROBE；iso2 与 ps 各 scan 一次；
#   B) 在 ps   起长生命周期节点 /DEMO_UNIQUE_PROBE；iso1 与 iso2 各 scan 一次。
# 预期（隔离生效）：iso1/iso2 互相看不到、也看不到 ps 的探针；
#                  而 ps（不隔离）能"看到" iso1 的探针。
# =============================================================================
set -uo pipefail
FOXY=/media/step/data/Yccc7/ParkSim-JTH/deps/ros/foxy
WS=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/workspace/install

run() { # run <container> <args...>
    local c=$1; shift
    docker exec "$c" bash -c "source $FOXY/setup.bash >/dev/null 2>&1; source $WS/setup.bash >/dev/null 2>&1; python3 /tmp/uniq_probe.py $*" 2>&1 \
        | grep -vE "DeprecationWarning|pkg_resources"
}

echo "================ A) iso1 起探针，iso2 / ps 观察 ================"
docker exec -d iso1 bash -c "source $FOXY/setup.bash >/dev/null 2>&1; source $WS/setup.bash >/dev/null 2>&1; python3 /tmp/uniq_probe.py hold ISO1 45 > /tmp/hold.log 2>&1"
sleep 6
run iso2 scan ISO2
run ps scan DEMO
echo "--- iso1 自见（自身探针应可见）---"
run iso1 scan ISO1

echo
echo "================ B) ps(demo, 不隔离) 起探针，iso1 / iso2 观察 ================"
docker exec -d ps bash -c "source $FOXY/setup.bash >/dev/null 2>&1; source $WS/setup.bash >/dev/null 2>&1; python3 /tmp/uniq_probe.py hold DEMO 45 > /tmp/hold.log 2>&1"
sleep 6
run iso1 scan ISO1
run iso2 scan ISO2

echo
echo "================ 各容器 ROS_LOCALHOST_ONLY 实际值 ================"
for c in iso1 iso2 ps; do
    echo "  $c: ROS_LOCALHOST_ONLY=$(docker exec "$c" printenv ROS_LOCALHOST_ONLY 2>/dev/null || echo '<unset>=0')"
done
