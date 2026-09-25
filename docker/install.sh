#!/usr/bin/env bash
# =============================================================================
# ParkSim-JTH 一体化镜像 —— 目标机一键安装 / 启动 / 运维脚本
#
#   用法：
#     ./install.sh                      # 载入镜像并后台启动（宿主端口 8098，默认停在「已停止」态）
#     ./install.sh --port 9000          # 指定宿主端口
#     ./install.sh --map jth_b1         # 指定地图
#     ./install.sh --autostart          # 开机即自动发车（等价 -e PARKSIM_AUTOSTART=1）
#     ./install.sh --fg                 # 前台运行（Ctrl+C 退出，日志直出）
#
#   运维：
#     ./install.sh --status             # 容器状态 + 侦听端口 + 镜像里的构建版本（哪个 commit）
#     ./install.sh --logs [N]           # 跟踪日志（默认从末尾 200 行起）
#     ./install.sh --version            # 只打印镜像 BUILDINFO（源码分支/commit/构建时间）
#     ./install.sh --check              # 只做交付内容自检（不启动）
#     ./install.sh --stop               # 停止并删除容器
#     ./install.sh --uninstall          # 停止容器、删镜像，并回收悬空镜像层
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
LOGS_TAIL="200"
ACTION="install"

# 镜像内 webviz 静态资源目录（与 Dockerfile 中的 PARKSIM_ROOT 保持一致）
IMAGE_STATIC_DIR="/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/webviz/static"
IMAGE_BUILDINFO="/opt/parksim/BUILDINFO"
# 交付内容红线：这 5 个前端文件缺任何一个，页面就是旧的（曾交付过只有 3 个文件的镜像）
FRONTEND_FILES="index.html app.js style.css charts.js charts.css"

# DDS 隔离：默认 1（可被外部环境变量覆盖，用于多机联调）
ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
# 自动发车：默认 0（就绪后停在「已停止」态，由页面「开启仿真」触发）
PARK_AUTOSTART="${PARKSIM_AUTOSTART:-0}"

usage() {
    # 打印文件开头的整段注释作为帮助（不依赖硬编码行号，改头部不会让帮助错位）
    awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"
}

while [ $# -gt 0 ]; do
    case "$1" in
        --port)      HOST_PORT="$2"; shift 2 ;;
        --map)       PARK_MAP="$2"; shift 2 ;;
        --tar)       IMAGE_TAR="$2"; shift 2 ;;
        --autostart) PARK_AUTOSTART="1"; shift ;;
        --fg)        FOREGROUND="yes"; shift ;;
        --status)    ACTION="status"; shift ;;
        --version)   ACTION="version"; shift ;;
        --check)     ACTION="check"; shift ;;
        --logs)      ACTION="logs"
                     if [ -n "${2:-}" ] && [ "${2#--}" = "${2}" ]; then LOGS_TAIL="$2"; shift; fi
                     shift ;;
        --stop)      docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
                     echo "[install] 容器 ${CONTAINER_NAME} 已停止并删除"; exit 0 ;;
        --uninstall) docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true
                     docker rmi -f "${IMAGE_NAME}" >/dev/null 2>&1 || true
                     # rmi 只删标签，镜像层仍在磁盘上；不 prune 的话一次卸载会留下 1.2G 悬空层
                     docker image prune -f >/dev/null 2>&1 || true
                     echo "[install] 已删除容器与镜像 ${IMAGE_NAME}（并回收悬空镜像层）"
                     exit 0 ;;
        -h|--help)   usage; exit 0 ;;
        *)           echo "[install] 未知参数：$1（用 --help 看用法）"; exit 2 ;;
    esac
done

command -v docker >/dev/null 2>&1 || { echo "[install] 未找到 docker，请先安装 Docker"; exit 1; }

# ---- 辅助 -------------------------------------------------------------------
image_exists() { docker image inspect "${IMAGE_NAME}" >/dev/null 2>&1; }

# 镜像里的构建溯源信息（老镜像没有该文件 → 如实说明，不假装知道）
image_buildinfo() {
    docker run --rm --entrypoint cat "${IMAGE_NAME}" "${IMAGE_BUILDINFO}" 2>/dev/null \
        || echo "（该镜像无 ${IMAGE_BUILDINFO}，无法确定构建版本——请用 docker/build_image.sh 重建）"
}

# 交付内容自检：前端 5 件齐 + index.html 真的挂载了 charts
# 交付内容自检：前端 5 件齐 + index.html 真的挂载了 charts
# 为什么必须有这一步：曾交付过一个 webviz/static 只有 app.js/index.html/style.css 的
# 镜像——镜像能 load、容器能起、install.sh 也能报「就绪」，用户看到的却是没有图表
# 模块的旧页面。不在这里拦住，这种「静默装旧版」只能靠人工比对才发现。
# 说明：容器内脚本刻意用单引号包裹、文件名与路径写字面量，避免外层变量被提前展开。
assert_image_contents() {
    local out rc
    out="$(docker run --rm --entrypoint bash "${IMAGE_NAME}" -c '
S=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/webviz/static
miss=""
for f in index.html app.js style.css charts.js charts.css; do
    [ -f "$S/$f" ] || miss="$miss $f"
done
if [ -n "$miss" ]; then echo "MISSING:$miss"; exit 10; fi
grep -q "charts[.]js"  "$S/index.html" || { echo "index.html 未引用 charts.js";   exit 11; }
grep -q "charts[.]css" "$S/index.html" || { echo "index.html 未引用 charts.css";  exit 12; }
grep -q "metricsViz"   "$S/index.html" || { echo "index.html 缺 #metricsViz";     exit 13; }
grep -q "eventsViz"    "$S/index.html" || { echo "index.html 缺 #eventsViz";      exit 14; }
echo OK
' 2>&1)" && rc=0 || rc=$?
    if [ "${rc}" -ne 0 ]; then
        echo "[install][FATAL] 镜像交付内容自检未通过（前端应为：${FRONTEND_FILES}）："
        echo "${out}" | sed 's/^/           /'
        echo "[install] 该镜像的 webviz 前端不完整或为旧版，装上去页面会缺图表模块。"
        echo "[install] 请用仓库里的 docker/build_image.sh 从当前 HEAD 重新构建镜像。"
        exit 1
    fi
    echo "[install] 交付内容自检 OK（前端 5 件齐全且已挂载 charts）"
}

# tar 完整性校验：有 .sha256 就严格校验，没有就提示
verify_tarball() {
    local sha="${IMAGE_TAR}.sha256"
    if [ ! -f "${IMAGE_TAR}" ]; then return 0; fi
    if [ -f "${sha}" ]; then
        echo "[install] 校验 sha256（$(basename "${IMAGE_TAR}")，约 376 MB，需十几秒）..."
        if ( cd "$(dirname "${IMAGE_TAR}")" && sha256sum -c "$(basename "${sha}")" >/dev/null 2>&1 ); then
            echo "[install] sha256 校验通过"
        else
            echo "[install][FATAL] sha256 校验失败：${IMAGE_TAR} 已损坏（传输/拷贝不完整）。"
            echo "[install] 请重新获取该文件及其 .sha256 后重试。"
            exit 1
        fi
    else
        echo "[install][WARN] 未找到 ${sha}，跳过完整性校验（无法发现 tar 损坏）。"
    fi
}

# 宿主端口占用预检：docker run -p 撞端口时报错信息很不友好，这里提前拦
port_in_use() {
    local p="$1"
    if command -v ss >/dev/null 2>&1; then
        ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${p}\$" && return 0
    elif command -v netstat >/dev/null 2>&1; then
        netstat -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${p}\$" && return 0
    elif command -v lsof >/dev/null 2>&1; then
        lsof -nP -iTCP:"${p}" -sTCP:LISTEN >/dev/null 2>&1 && return 0
    fi
    return 1
}

# 宿主 IP：让用户拿到就能点，而不是面对 <目标机IP> 自己猜
host_ip() {
    local ip=""
    if command -v hostname >/dev/null 2>&1; then
        ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
    fi
    if [ -z "${ip}" ] && command -v ip >/dev/null 2>&1; then
        ip="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)"
    fi
    [ -n "${ip}" ] || ip="127.0.0.1"
    printf '%s' "${ip}"
}

# ---- 运维子命令（不需要先载入镜像）-----------------------------------------
case "${ACTION}" in
    version)
        image_exists || { echo "[install] 镜像 ${IMAGE_NAME} 不存在；先跑 ./install.sh 载入"; exit 1; }
        echo "[install] 镜像 ${IMAGE_NAME} 的构建溯源："
        image_buildinfo | sed 's/^/           /'
        exit 0 ;;
    check)
        image_exists || { echo "[install] 镜像 ${IMAGE_NAME} 不存在；先跑 ./install.sh 载入"; exit 1; }
        assert_image_contents
        echo "[install] 溯源："
        image_buildinfo | sed 's/^/           /'
        exit 0 ;;
    status)
        if ! image_exists; then
            echo "[install] 镜像 ${IMAGE_NAME}：不存在"
        else
            echo "[install] 镜像 ${IMAGE_NAME}：已载入（$(docker image inspect -f '{{.Size}}' "${IMAGE_NAME}" | awk '{printf "%.2f GB", $1/1024/1024/1024}')）"
        fi
        if docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}"; then
            docker ps -a --filter "name=^${CONTAINER_NAME}$" \
                --format '[install] 容器：{{.Status}}  端口：{{.Ports}}  镜像：{{.Image}}'
        else
            echo "[install] 容器 ${CONTAINER_NAME}：不存在"
        fi
        echo "[install] 侦听端口 ${HOST_PORT}：$(port_in_use "${HOST_PORT}" && echo '已被占用' || echo '空闲')"
        echo "[install] 版本："
        image_buildinfo | sed 's/^/           /'
        exit 0 ;;
    logs)
        docker ps -a --format '{{.Names}}' | grep -qx "${CONTAINER_NAME}" \
            || { echo "[install] 容器 ${CONTAINER_NAME} 不存在"; exit 1; }
        exec docker logs -f --tail "${LOGS_TAIL}" "${CONTAINER_NAME}" ;;
esac

# ---- 1) 载入镜像（tar 存在且镜像尚未载入时）--------------------------------
if ! image_exists; then
    if [ -f "${IMAGE_TAR}" ]; then
        verify_tarball
        echo "[install] 载入镜像：${IMAGE_TAR}（解压后约 1.3 GB，可能需要 1-3 分钟）"
        gzip -dc "${IMAGE_TAR}" | docker load
    else
        echo "[install] 镜像 ${IMAGE_NAME} 不存在，且未找到 ${IMAGE_TAR}"
        exit 1
    fi
else
    echo "[install] 镜像 ${IMAGE_NAME} 已存在，跳过 load"
fi

# ---- 1b) 交付内容自检（挡住"镜像能装、页面却是旧版"这种静默失败）-----------
assert_image_contents
echo "[install] 镜像构建版本："
image_buildinfo | sed 's/^/           /'

# ---- 2) 启动容器 -----------------------------------------------------------
docker rm -f "${CONTAINER_NAME}" >/dev/null 2>&1 || true

# 端口预检：容器名已删，此时若端口仍被占，多半是别的服务
if port_in_use "${HOST_PORT}"; then
    echo "[install][FATAL] 宿主端口 ${HOST_PORT} 已被占用，无法映射。"
    echo "[install] 换端口重试：./install.sh --port 9000"
    echo "[install] 查占用：ss -ltnp | grep ${HOST_PORT}"
    exit 1
fi

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

IP="$(host_ip)"
echo "[install] 容器已启动：${CONTAINER_NAME}"
echo "[install]   页面     http://${IP}:${HOST_PORT}/   （本机 http://127.0.0.1:${HOST_PORT}/）"
echo "[install]   看日志   ./install.sh --logs   （或 docker logs -f ${CONTAINER_NAME}）"
echo "[install]   看状态   ./install.sh --status"
echo "[install]   看版本   ./install.sh --version"
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
    echo "[install] （未找到 curl/wget，跳过就绪探测；请自行访问 http://${IP}:${HOST_PORT}/）"
    exit 0
fi

printf '[install] 等待 webviz 就绪'
for i in $(seq 1 30); do
    if ${PROBE_CMD} >/dev/null 2>&1; then
        echo
        echo "[install] ✅ 就绪：http://${IP}:${HOST_PORT}/"
        exit 0
    fi
    printf '.'
    sleep 2
done
echo
echo "[install] ⚠️ 60s 内未探测到 HTTP 响应，请查看：docker logs ${CONTAINER_NAME}"
exit 1
