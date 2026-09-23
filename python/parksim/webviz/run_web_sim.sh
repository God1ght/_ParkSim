#!/usr/bin/env bash
# ParkSim 一键启动：webviz 桥（托管模式：网页可换方案并重启）+ 仿真
#
# 用法（先 source 环境）：
#   source /media/step/data/Yccc7/ParkSim-JTH/env_ros.sh
#   python/parksim/webviz/run_web_sim.sh [端口，默认 8099] [launch 参数...]
#
# 说明：
#   - 先清理旧的桥/仿真进程；桥以托管模式启动（--control --manage-sim），由桥负责启动仿真；
#   - 默认「底图模式」：桥启动后只加载底图，仿真由网页上的「开启仿真」按钮启动（与底图解耦）；
#   - 如需桥启动时自动开仿真：WEBVIZ_AUTO_START=1 run_web_sim.sh ...
#   - 网页底部「方案」面板可切换 场景/泊位分配/路由/参考路径/机动 并一键重启（无需回终端）；
#   - 附加 launch 参数作为初始方案，如：run_web_sim.sh 8099 allocation_method:=graph_cost
set -e

# 提高文件描述符上限（ROS2 Foxy 多车辆 DDS 发现会消耗大量 fd；
# 默认 1024 在多车场景下可能耗尽导致节点异常退出）
ulimit -n 65536 2>/dev/null || ulimit -n 8192 2>/dev/null || true

PORT="${1:-8099}"
if [ $# -gt 0 ]; then shift; fi
EXTRA_ARGS=("$@")
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$HERE/../../.." && pwd)"
export PARKSIM_ROOT="${PARKSIM_ROOT:-$REPO_ROOT}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[webviz] 未检测到 ros2；请先 source ROS 环境（如 env_ros.sh）" >&2
  exit 1
fi

echo "[webviz] repo root: $REPO_ROOT"
echo "[webviz] 清理旧的仿真/桥进程（如有）..."
# 注意：使用 SIGKILL(-9) 强制清理——避免旧桥在被终止时执行延迟 sweep，
# 误杀刚启动的新仿真实例（历史事故根因）。
pkill -9 -f "ros2 launch parksim" 2>/dev/null || true
pkill -9 -f "parksim.*simulator_node" 2>/dev/null || true
pkill -9 -f "parksim.*vehicle_node" 2>/dev/null || true
pkill -9 -f "webviz/server.py" 2>/dev/null || true
sleep 2

if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
  echo "[webviz] 初始方案参数: ${EXTRA_ARGS[*]}"
fi
echo "[webviz] 启动桥（端口 $PORT，托管仿真，网页可换方案重启）..."
AUTO_FLAG=""
if [ "${WEBVIZ_AUTO_START:-0}" = "1" ]; then AUTO_FLAG="--auto-start"; fi
exec python -u "$HERE/server.py" --port "$PORT" --control --manage-sim $AUTO_FLAG --launch-args "${EXTRA_ARGS[*]}"
