#!/usr/bin/env bash
# =============================================================================
# ParkSim-JTH 一体化镜像入口
#   source 镜像内置 ROS 2 Foxy 树 + workspace/install → 启动 webviz 桥（含托管仿真）
#   所有输出走 stdout/stderr，直接 `docker logs -f <name>` 查看
#
# 可用环境变量：
#   PORT                监听端口（容器内，默认 8099）
#   PARKSIM_MAP         地图名（默认 jth_b1）
#   ROS_DOMAIN_ID       DDS 域（默认 0；容器自带网络命名空间，天然与宿主隔离）
#   ROS_LOCALHOST_ONLY  置 1 可把 DDS 限制在容器 lo
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

PORT="${PORT:-8099}"
MAP="${PARKSIM_MAP:-jth_b1}"

# --- 激活内置 ROS 2 Foxy（非 /opt/ros）与 colcon 安装空间 ---
set +u
source "${ROS_FOXY_ROOT}/setup.bash"
source "${PARKSIM_ROOT}/workspace/install/setup.bash"
set -u
# dlp 在构建机是 develop 安装（egg-link）；镜像里显式加入 PYTHONPATH
export PYTHONPATH="${DLP_DATASET_ROOT}:${PYTHONPATH:-}"

echo "[run_app] PARKSIM_ROOT=${PARKSIM_ROOT}"
echo "[run_app] ROS_DISTRO=${ROS_DISTRO:-?}  ros2=$(command -v ros2 || echo MISSING)"
echo "[run_app] python3=$(command -v python3) ($(python3 -V 2>&1))"
echo "[run_app] map=${MAP}  listen=0.0.0.0:${PORT}"

cd "${PARKSIM_ROOT}/python"
exec python3 -u "${PARKSIM_ROOT}/python/parksim/webviz/server.py" \
    --host 0.0.0.0 \
    --port "${PORT}" \
    --control \
    --manage-sim \
    --auto-start \
    --launch-args "map:=${MAP}" \
    "$@"
