#!/usr/bin/env bash
# =============================================================================
# ParkSim-JTH 一体化镜像入口
#   source 镜像内置 ROS 2 Foxy 树 + workspace/install → 启动 webviz 桥（含托管仿真）
#   所有输出走 stdout/stderr，直接 `docker logs -f <name>` 查看
#
# 可用环境变量：
#   PORT                监听端口（容器内，默认 8099）
#   PARKSIM_MAP         地图名（默认 jth_b1）
#   PARKSIM_AUTOSTART   是否开机即自动发车。默认 **0 = 停在「已停止」态**，
#                       由用户在网页点「开启仿真」发车（符合「打开页面按键控制」的需求）。
#                       取 1/true/yes/on 时追加 --auto-start，容器一起来就开跑。
#   ROS_DOMAIN_ID       DDS 域（默认 0）
#   ROS_LOCALHOST_ONLY  默认 1（镜像内已固化）。**重要**：bridge 网络**并不能**隔离
#                       DDS 发现——实测容器与宿主/其它容器的 DDS 会双向互相发现，
#                       多实例并存时会发现外来 vehicle 节点，导致本容器启动即退出。
#                       置 1 让 DDS 只走容器自身 lo，彻底避免串扰。仅在需要与外部 ROS
#                       节点互相发现（多机联调）时才显式覆盖为 0，并自行承担风险。
# =============================================================================
# 注意：source ROS/ament 的 setup.bash 期间必须关闭 nounset（-u）。
# ament 的 setup 脚本会引用 AMENT_TRACE_SETUP_FILES 等未定义变量，
# 在 `set -u` 下会直接以 "unbound variable" 退出（构建期已踩过同一坑）。
set -eo pipefail

export PARKSIM_ENV_ROOT="${PARKSIM_ENV_ROOT:-/media/step/data/Yccc7/ParkSim-JTH}"
export PARKSIM_ROOT="${PARKSIM_ROOT:-${PARKSIM_ENV_ROOT}/_ParkSim}"
export PARKSIM_DATA_ROOT="${PARKSIM_DATA_ROOT:-${PARKSIM_ROOT}/python/parksim/priorFiles}"
export ROS_FOXY_ROOT="${ROS_FOXY_ROOT:-${PARKSIM_ENV_ROOT}/deps/ros/foxy}"
export DLP_DATASET_ROOT="${DLP_DATASET_ROOT:-${PARKSIM_ENV_ROOT}/deps/dlp-dataset}"

# DDS 隔离兜底：Dockerfile 已用 ENV 固化，这里再兜一层（外部 -e 可覆盖）
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

PORT="${PORT:-8099}"
MAP="${PARKSIM_MAP:-jth_b1}"

# 自动发车开关：默认关闭。用户需求是「打开页面，按键开启/关闭仿真」，
# 故就绪后应停在「已停止」态，由用户点「开启仿真」触发，而不是替用户发车。
AUTOSTART="${PARKSIM_AUTOSTART:-0}"

# --- 激活内置 ROS 2 Foxy（非 /opt/ros）与 colcon 安装空间 ---
set +u
source "${ROS_FOXY_ROOT}/setup.bash"
source "${PARKSIM_ROOT}/workspace/install/setup.bash"
set -u
# dlp 在构建机是 develop 安装（egg-link）；镜像里显式加入 PYTHONPATH
export PYTHONPATH="${DLP_DATASET_ROOT}:${PYTHONPATH:-}"

# --- 组装 server.py 参数（--auto-start 仅在开关打开时追加） ---
server_args=(
    --host 0.0.0.0
    --port "${PORT}"
    --control
    --manage-sim
    --launch-args "map:=${MAP}"
)
AUTOSTART_ON=0
case "${AUTOSTART}" in
    1|true|True|TRUE|yes|Yes|YES|on|ON) AUTOSTART_ON=1; server_args+=(--auto-start) ;;
esac

echo "[run_app] PARKSIM_ROOT=${PARKSIM_ROOT}"
echo "[run_app] ROS_DISTRO=${ROS_DISTRO:-?}  ros2=$(command -v ros2 || echo MISSING)"
echo "[run_app] python3=$(command -v python3) ($(python3 -V 2>&1))"
echo "[run_app] map=${MAP}  listen=0.0.0.0:${PORT}"
echo "[run_app] autostart=${AUTOSTART_ON} (PARKSIM_AUTOSTART=${AUTOSTART})"
echo "[run_app] ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"

cd "${PARKSIM_ROOT}/python"
exec python3 -u "${PARKSIM_ROOT}/python/parksim/webviz/server.py" "${server_args[@]}" "$@"
