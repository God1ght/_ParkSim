#!/usr/bin/env bash
# =============================================================================
# ParkSim-JTH 一体化镜像 —— 目标机一键安装 / 启动脚本
#
#   用法：
#     ./install.sh                      # 载入镜像并后台启动（宿主端口 8098，默认停在「已停止」态）
#     ./install.sh --port 9000          # 指定宿主端口
#     ./install.sh --map jth_b1         # 指定地图
#     ./install.sh --autostart          # 开机即自动发车（等价 -e PARKSIM_AUTOSTART=1）
#     ./install.sh --stop               # 停止并删除容器
#     ./install.sh --uninstall          # 停止容器并删除镜像
#     ./install.sh --fg                 # 前台运行（Ctrl+C 退出，日志直出）
#
#   前置要求：目标机已安装 Docker（>=19.03），无需 ROS / conda / 源码。
#
#   自动发车（PARKSIM_AUTOSTART）：默认 0 —— webviz 就绪后停在「已停止」态，
#   由用户在网页点「开启仿真」发车；需要开机即跑时传 --autostart 或
#   环境变量 PARKSIM_AUTOSTART=1。
#
#   DDS 隔离（重要）：镜像内已固化 ROS_LOCALHOST_ONLY=1。默认 bridge 网络
#   **并不能**隔离 DDS 发现——实测容器 DDS 会与宿主/其它容器的 DDS 双向互相
#   发现，多实例并存时会发现外来 vehicle 节点，导致本容器启动即退出(1)。
#   本脚本默认显式传 -e ROS_LOCALHOST_ONLY=1（与镜像默认一致，双保险）。
#   仅在需要与外部 ROS 节点互相发现（多机联调）时，才用
#     ROS_LOCALHOST_ONLY=0 ./install.sh ...
#   覆盖，并自行承担发现串扰。
# =============================================================================
set -eo pipefail

IMAGE_NAME="parksim-jth:v1"
IMAGE_TAR="$(cd "$(dirname "$0")" && pwd)/parksim-jth-v1.tar.gz"
CONTAINER_NAME="parksim-jth"
HOST_PORT="8098"
CONTAINER_PORT="8099"
PARK_MAP="jth_b1"
FOREGROUND="no"

# DDS 隔离：默认 1（可被外部环境变量覆盖，用于多机联调）
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# 自动发车：默认 0（就绪后停在「已停止」态，由页面「开启仿真」触发）
PARK_AUTOSTART="${PARKSIM_AUTOSTART:-0}"

while [ $# -gt 0 ]; do
    case "$1" in
        --port)      HOST_PORT="$2"; shift 2 ;;
        --map)       PARK_MAP="$2"; shift 2 ;;
        --tar)       IMAGE_TAR="$2"; shift 2 ;;
        --autostart) PARK_AUTOSTART="1"; shift ;;
        --stop)      docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
                     echo "[install] 容器 ${CONTAINER_NAME} 已停止并删除"; exit 0 ;;
        --uninstall) docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
                     docker rmi -f "${IMAGE_NAME}" >/dev/null 2>&1 || true
                     echo "[install] 已删除容器与镜像 ${IMAGE_NAME}"; exit 0 ;;
        --fg)        FOREGROUND="yes"; shift ;;
        -h|--help)   sed -n '2,28p' "$0"; exit 0 ;;
        *)           echo "[install] 未知参数：$1"; exit 2 ;;
    esac
done

command -v docker >/dev/null 2>&1 || { echo "[install] 未找到 docker，请先安装 Docker"; exit 1; }

# ---- 1) 载入镜像（tar 存在且镜像尚未载入时）--------------------------------
if ! docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
    if [ -f "${IMAGE_TAR}" ]; then
        echo "[install] 载入镜像：${IMAGE_TAR}（约 1.5 GB，可能需要 1-3 分钟）"
        gzip -dc "${IMAGE_TAR}" | docker load
    else
        echo "[install] 镜像 ${IMAGE_NAME} 不存在，且未找到 ${IMAGE_TAR}"
        exit 1
    fi
else
    echo "[install] 镜像 ${IMAGE_NAME} 已存在，跳过 load"
fi

# ---- 2) 启动容器 -----------------------------------------------------------
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

if [ "${FOREGROUND}" = "yes" ]; then
    echo "[install] 前台启动（Ctrl+C 退出）"
    exec docker run --rm --name "${CONTAINER_NAME}" \
        -p "${HOST_PORT}:${CONTAINER_PORT}" \
        -e PORT="${CONTAINER_PORT}" \
        -e PARKSIM_MAP="${PARK_MAP}" \
        -e PARKSIM_AUTOSTART="${PARK_AUTOSTART}" \
        -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
        "${IMAGE_NAME}"
fi

docker run -d --name "${CONTAINER_NAME}" \
    --restart unless-stopped \
    -p "${HOST_PORT}:${CONTAINER_PORT}" \
    -e PORT="${CONTAINER_PORT}" \
    -e PARKSIM_MAP="${PARK_MAP}" \
    -e PARKSIM_AUTOSTART="${PARK_AUTOSTART}" \
    -e ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY}" \
    "${IMAGE_NAME}" >/dev/null

echo "[install] 容器已启动：${CONTAINER_NAME}"
echo "[install]   页面     http://<目标机IP>:${HOST_PORT}/"
echo "[install]   看日志   docker logs -f ${CONTAINER_NAME}"
echo "[install]   停止     ./install.sh --stop"
echo "[install]   提示：默认停在「已停止」态，请在网页点「开启仿真」发车；"
echo "[install]         若需开机即跑，用 ./install.sh --autostart（PARKSIM_AUTOSTART=1）。"
echo "[install]   DDS      ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY}（1=只走容器 lo，避免跨实例串扰）"
echo "[install]   AUTOSTART=${PARK_AUTOSTART}（0=就绪后不自动发车）"

# ---- 3) 就绪探测（最多 60s，缺少 curl/wget 时跳过）-------------------------
PROBE_CMD=""
if command -v curl >/dev/null 2>&1; then
    PROBE_CMD="curl -fsS http://127.0.0.1:${HOST_PORT}/"
elif command -v wget >/dev/null 2>&1; then
    PROBE_CMD="wget -q -O /dev/null http://127.0.0.1:${HOST_PORT}/"
fi

if [ -z "${PROBE_CMD}" ]; then
    echo "[install] （未找到 curl/wget，跳过就绪探测；请自行访问 http://<目标机IP>:${HOST_PORT}/）"
    exit 0
fi

printf '[install] 等待 webviz 就绪'
for i in $(seq 1 30); do
    if ${PROBE_CMD} >/dev/null 2>&1; then
        echo
        echo "[install] ✅ 就绪：http://127.0.0.1:${HOST_PORT}/"
        exit 0
    fi
    printf '.'
    sleep 2
done
echo
echo "[install] ⚠️ 60s 内未探测到 HTTP 响应，请查看：docker logs ${CONTAINER_NAME}"
exit 1
