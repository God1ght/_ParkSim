#!/usr/bin/env bash
# =============================================================================
# DDS 隔离性验证（ROS_LOCALHOST_ONLY=1 修复后）
#
# 前提（保持不动）：宿主 8099 实例在跑；8098 demo 容器 `ps`（旧镜像、无隔离）在跑。
# 场景：连续 3 轮，每轮起两个「默认参数」容器 iso1(8096) / iso2(8097)，
#       要求全部稳定启动、不退出、日志无污染标记。
# 之后：对两个容器各发一次 restart（入2出2）冒烟；再比对 node list 验证互不可见。
# =============================================================================
set -uo pipefail
IMG=parksim-jth:v1
FOXY=/media/step/data/Yccc7/ParkSim-JTH/deps/ros/foxy
WS=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/workspace/install

echo "### 环境（应看到宿主 8099 + 8098 demo 都在）"
ps -p 731123 -o pid,etime,cmd 2>/dev/null | tail -1
docker ps --format "  {{.Names}} {{.Status}} {{.Ports}} {{.Image}}"

FAIL=0
for round in 1 2 3; do
    echo
    echo "================ ROUND $round ================"
    docker rm -f iso1 iso2 >/dev/null 2>&1
    docker run -d --name iso1 -p 8096:8099 "$IMG" >/dev/null
    docker run -d --name iso2 -p 8097:8099 "$IMG" >/dev/null
    sleep 30
    for c in iso1 iso2; do
        state=$(docker inspect -f '{{.State.Status}}' "$c")
        code=$(docker inspect -f '{{.State.ExitCode}}' "$c")
        env_ok=$(docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' "$c" | grep -c '^ROS_LOCALHOST_ONLY=1$')
        own=$(docker logs "$c" 2>&1 | grep -c '\[webviz\] serving on http://0.0.0.0:8099/')
        bad=$(docker logs "$c" 2>&1 | grep -cE 'not shut down cleanly|exited early|Traceback \(most recent' || true)
        echo "  $c: state=$state exit=$code  ENV_ROS_LOCALHOST_ONLY=1:$env_ok  webviz_serving:$own  contamination:$bad"
        if [ "$state" != "running" ] || [ "$code" != "0" ] || [ "$own" -lt 1 ] || [ "$bad" -ne 0 ]; then
            FAIL=$((FAIL + 1))
            echo "    ^^^ ROUND $round $c 判定失败；日志尾部："
            docker logs "$c" 2>&1 | tail -12 | sed 's/^/      /'
        fi
    done
done

echo
echo "### 3 轮并发启动结论：FAIL_COUNT=$FAIL（0 = 全部稳定）"

echo
echo "### 各容器自见节点（隔离正常时不应出现「外来」vehicle）"
for c in iso1 iso2; do
    echo "--- $c node list ---"
    docker exec "$c" bash -lc "source $FOXY/setup.bash >/dev/null 2>&1; source $WS/setup.bash >/dev/null 2>&1; ros2 node list 2>/dev/null | sort" | sed 's/^/    /'
done

echo
echo "### 8098 demo（旧镜像，故意不隔离）node list —— 作为反例"
docker exec ps bash -lc "source $FOXY/setup.bash >/dev/null 2>&1; source $WS/setup.bash >/dev/null 2>&1; ros2 node list 2>/dev/null | sort" | sed 's/^/    /'

echo
echo "### 容器内 ROS_LOCALHOST_ONLY 实测值"
for c in iso1 iso2 ps; do
    v=$(docker exec "$c" printenv ROS_LOCALHOST_ONLY 2>/dev/null || echo "<unset>")
    echo "  $c: ROS_LOCALHOST_ONLY=$v"
done
exit $FAIL
