#!/usr/bin/env bash
# =============================================================================
# run.sh —— 启动 ParkSim-JTH webviz 可视化 + 仿真托管服务
#
#   等价于 docker/run_app.sh：导出环境 → source 内置 ROS 2 Foxy →
#   source workspace/install → python3 webviz/server.py
#
# 用法：
#   bash run.sh
#   PORT=8080 bash run.sh                 # 换端口（默认 8099）
#   PARKSIM_MAP=jth_b1 bash run.sh        # 换地图（默认 jth_b1）
#   PARKSIM_AUTOSTART=1 bash run.sh       # 起来就自动发车（默认停在「已停止」，由网页控制）
#
# 可用环境变量：
#   PARKSIM_ENV_ROOT    环境根（默认：本脚本所在目录；若该目录下没有 _ParkSim
#                       则回退 /media/step/data/Yccc7/ParkSim-JTH）
#   PORT                监听端口（默认 8099）
#   PARKSIM_MAP         地图名（默认 jth_b1）
#   PARKSIM_AUTOSTART   1/true/yes/on 时开机即发车（默认 0）
#   ROS_DOMAIN_ID       DDS 域（默认 0）
#   ROS_LOCALHOST_ONLY  DDS 只走 lo（默认 1，避免同机多实例互相发现导致启动即退出）
#
# 注意：**必须绑 0.0.0.0**。WSL2 是 NAT 模式，绑 127.0.0.1 会让 Windows 侧访问不到。
# =============================================================================
set -eo pipefail

CANONICAL_ROOT="/media/step/data/Yccc7/ParkSim-JTH"
SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -d "${SELF_DIR}/_ParkSim" ]; then
  PARKSIM_ENV_ROOT="${PARKSIM_ENV_ROOT:-${SELF_DIR}}"
else
  PARKSIM_ENV_ROOT="${PARKSIM_ENV_ROOT:-${CANONICAL_ROOT}}"
fi

export PARKSIM_ENV_ROOT
export PARKSIM_ROOT="${PARKSIM_ROOT:-${PARKSIM_ENV_ROOT}/_ParkSim}"
export PARKSIM_DATA_ROOT="${PARKSIM_DATA_ROOT:-${PARKSIM_ROOT}/python/parksim/priorFiles}"
export ROS_FOXY_ROOT="${ROS_FOXY_ROOT:-${PARKSIM_ENV_ROOT}/deps/ros/foxy}"
export DLP_DATASET_ROOT="${DLP_DATASET_ROOT:-${PARKSIM_ENV_ROOT}/deps/dlp-dataset}"

# DDS 隔离：只走本机 lo，避免同机/邻居机器的 ROS 节点互相发现造成串扰
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# 运行链路会 import matplotlib.pyplot（不弹窗，但保险）：强制无 GUI 后端
export MPLBACKEND="${MPLBACKEND:-Agg}"
export PYTHONUNBUFFERED=1

PORT="${PORT:-8099}"
MAP="${PARKSIM_MAP:-jth_b1}"
AUTOSTART="${PARKSIM_AUTOSTART:-0}"

die() { printf '\033[1;31m[run][FATAL]\033[0m %s\n' "$*" >&2; exit 1; }

[ -d "${PARKSIM_ROOT}" ]     || die "找不到 ${PARKSIM_ROOT} —— 先跑 install.sh（或设 PARKSIM_ENV_ROOT）"
[ -f "${ROS_FOXY_ROOT}/setup.bash" ] || die "找不到 ROS 2 Foxy 树：${ROS_FOXY_ROOT}/setup.bash"
[ -f "${PARKSIM_ROOT}/workspace/install/setup.bash" ] || die "找不到 colcon 产物：${PARKSIM_ROOT}/workspace/install/setup.bash"
[ -f "${PARKSIM_ROOT}/python/parksim/webviz/server.py" ] || die "找不到 webviz/server.py"

# --- 激活内置 ROS 2 Foxy（注意：不碰 /opt/ros）与 colcon 安装空间 ---
# ament 的 setup 脚本会引用 AMENT_TRACE_SETUP_FILES 等未定义变量，
# 在 set -u 下会直接以 "unbound variable" 退出，故 source 期间必须关掉 nounset。
set +u
# shellcheck disable=SC1090
source "${ROS_FOXY_ROOT}/setup.bash"
# shellcheck disable=SC1090
source "${PARKSIM_ROOT}/workspace/install/setup.bash"
set -u

# dlp 是纯源码包，显式加入 PYTHONPATH（构建机是 develop 安装，目标机没有 egg-link）
export PYTHONPATH="${DLP_DATASET_ROOT}:${PYTHONPATH:-}"

server_args=(
    --host 0.0.0.0
    --port "${PORT}"
    --control
    --manage-sim
    --launch-args "map:=${MAP}"
)
case "${AUTOSTART}" in
  1|true|True|TRUE|yes|Yes|YES|on|ON) server_args+=(--auto-start) ;;
esac

echo "[run] PARKSIM_ENV_ROOT=${PARKSIM_ENV_ROOT}"
echo "[run] PARKSIM_ROOT=${PARKSIM_ROOT}"
echo "[run] ROS_DISTRO=${ROS_DISTRO:-?}  ros2=$(command -v ros2 || echo MISSING)"
echo "[run] python3=$(command -v python3) ($(python3 -V 2>&1))"
echo "[run] map=${MAP}  listen=0.0.0.0:${PORT}  autostart=${AUTOSTART}"
echo "[run] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  MPLBACKEND=${MPLBACKEND}"

cd "${PARKSIM_ROOT}/python"
exec python3 -u "${PARKSIM_ROOT}/python/parksim/webviz/server.py" "${server_args[@]}" "$@"
