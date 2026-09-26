#!/usr/bin/env bash
# =============================================================================
# 对照组（保留容器，采集退出证据）：证明「bridge 不隔离 DDS」这个缺陷方向是真的。
#   ctlA / ctlB：ROS_LOCALHOST_ONLY=0（= 旧镜像行为），8094 / 8095
# 预期：它们会通过 DDS 发现到彼此 + 8098 demo + 宿主 8099 的外来 vehicle 节点，
#       进而 simulator_node 报 "Some vehicle nodes are not shut down cleanly"
#       → RuntimeError: simulator exited early → 容器退出(1)。
# =============================================================================
set -uo pipefail
IMG="${PARKSIM_IMAGE:-parksim-jth:v2}"

echo "### 起两个 ROS_LOCALHOST_ONLY=0 容器（= 旧行为）"
docker rm -f ctlA ctlB >/dev/null 2>&1
docker run -d --name ctlA -p 8094:8099 -e ROS_LOCALHOST_ONLY=0 "$IMG" >/dev/null
docker run -d --name ctlB -p 8095:8099 -e ROS_LOCALHOST_ONLY=0 "$IMG" >/dev/null
sleep 40

for c in ctlA ctlB; do
    echo
    echo "---------- $c ----------"
    echo "state: $(docker inspect -f '{{.State.Status}}  exit={{.State.ExitCode}}  started={{.State.StartedAt}}  finished={{.State.FinishedAt}}' "$c")"
    echo "ROS_LOCALHOST_ONLY: $(docker exec "$c" printenv ROS_LOCALHOST_ONLY 2>/dev/null || echo '<容器已退出>')"
    echo "污染标记统计（not shut down cleanly / exited early / 外来 vehicle）:"
    docker logs "$c" 2>&1 | grep -nE "not shut down cleanly|exited early|Some vehicle nodes|subscribed vehicle" | head -12 | sed 's/^/    /'
    echo "容器看到的 vehicle 编号（若大量非自建编号 => 跨容器发现）："
    docker logs "$c" 2>&1 | grep -oE "subscribed vehicle [0-9]+" | sort -u -k3 -n | head -12 | tr '\n' ' '
    echo
    echo "它自己实际创建的 vehicle 编号："
    docker logs "$c" 2>&1 | grep -oE "A vehicle with id = [0-9]+" | sort -u -k6 -n | head -12 | tr '\n' ' '
    echo
    echo "日志尾部："
    docker logs "$c" 2>&1 | tail -8 | sed 's/^/    /'
done

echo
echo "### 采集完成；保留 ctlA/ctlB 供复核，稍后清理"
