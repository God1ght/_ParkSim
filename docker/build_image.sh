#!/usr/bin/env bash
# =============================================================================
# build_image.sh —— 从 git archive HEAD 一键重建 parksim-jth 镜像与交付 tar.gz
#
# 为什么需要这个脚本：
#   原先 /media/step/data/parksim-image-build/payload/parksim 是**手工铺**的，
#   既没有构建脚本、也没有版本戳。结果没人能回答「这个 1.28G 的镜像到底是哪个
#   commit」，也没法一键重建——交付过一整轮「镜像里没有 charts.js/charts.css」的
#   陈旧前端就是这么发生的。
#   本脚本把整条链路做成可复现：git archive HEAD → payload → build → save → sha256。
#
# 用法（在任意装有 docker 的机器上，只要能读到 ENV_ROOT）：
#   bash build_image.sh                 # 全量：重建 payload + build + save + sha256
#   bash build_image.sh --no-save       # 只 build，不打 tar（快速验证）
#   bash build_image.sh --refresh-payload  # 连 foxy/dlp/dji/priorfiles 也重新铺一遍
#   bash build_image.sh --no-prune      # 构建后不回收 dangling 镜像
#   bash build_image.sh --strict-clean  # 工作区有未提交改动时直接失败（默认只警告）
#   bash build_image.sh --tag parksim-jth:v2
#
# 前置：目标机已装 docker，且 ENV_ROOT 下存在 _ParkSim（git 仓库）、
#       deps/ros/foxy（内置 ROS 2 Foxy 树）、deps/dlp-dataset/dlp。
# =============================================================================
set -eo pipefail

ENV_ROOT="/media/step/data/Yccc7/ParkSim-JTH"
SRC_REPO="${ENV_ROOT}/_ParkSim"
FOXY="${ENV_ROOT}/deps/ros/foxy"
DLP="${ENV_ROOT}/deps/dlp-dataset"
BUILD_CTX="/media/step/data/parksim-image-build"
OUT_DIR="/media/step/data/ParkSim-JTH-image"
IMAGE="parksim-jth:v1"
TARBALL=""
PAYLOAD_ROOT="${BUILD_CTX}/payload"
STATIC_REL="python/parksim/webviz/static"
FRONTEND_FILES="index.html app.js style.css charts.js charts.css"

DO_SAVE=1
DO_PRUNE=1
REFRESH_PAYLOAD=0
STRICT_CLEAN=0

while [ $# -gt 0 ]; do
  case "$1" in
    --no-save)         DO_SAVE=0; shift ;;
    --no-prune)        DO_PRUNE=0; shift ;;
    --refresh-payload) REFRESH_PAYLOAD=1; shift ;;
    --strict-clean)    STRICT_CLEAN=1; shift ;;
    --tag)             IMAGE="$2"; shift 2 ;;
    --tag=*)           IMAGE="${1#--tag=}"; shift ;;
    --ctx)             BUILD_CTX="$2"; PAYLOAD_ROOT="${BUILD_CTX}/payload"; shift 2 ;;
    --out)             OUT_DIR="$2"; shift 2 ;;
    -h|--help)         awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"; exit 0 ;;
    *) echo "未知参数: $1" >&2; exit 2 ;;
  esac
done

[ -n "${TARBALL}" ] || TARBALL="${OUT_DIR}/$(echo "${IMAGE}" | tr ':' '-').tar.gz"

log()  { printf '\033[1;34m[build]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[build][WARN]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[build][FATAL]\033[0m %s\n' "$*" >&2; exit 1; }

command -v docker >/dev/null 2>&1 || die "未找到 docker"
[ -d "${SRC_REPO}/.git" ] || die "源码仓库不存在或不是 git 仓库: ${SRC_REPO}"
[ -d "${FOXY}" ]          || die "ROS 2 Foxy 树不存在: ${FOXY}"
[ -d "${DLP}/dlp" ]       || die "dlp 包不存在: ${DLP}/dlp"
mkdir -p "${BUILD_CTX}" "${OUT_DIR}" "${PAYLOAD_ROOT}"

GIT_HEAD="$(git -C "${SRC_REPO}" rev-parse --short HEAD)"
GIT_BRANCH="$(git -C "${SRC_REPO}" rev-parse --abbrev-ref HEAD)"
BUILD_TIME="$(date '+%Y-%m-%d %H:%M:%S %z')"

# 镜像内容 = git archive HEAD，与 HEAD 逐字节一致；工作区脏**不会**污染镜像，
# 只说明"你以为在跑的那份代码"和镜像可能不同（本机 webviz_runtime.yaml 会被
# 运行时回写，长期处于脏状态，所以默认不能因此拒绝构建）。
# 默认只警告并列出差异文件；需要绝对严格时用 --strict-clean 变成硬失败。
DIRTY="$(git -C "${SRC_REPO}" status --porcelain --untracked-files=no)"
if [ -n "${DIRTY}" ]; then
  if [ "${STRICT_CLEAN}" = "1" ]; then
    die "工作区有未提交改动（--strict-clean 已开启）：
${DIRTY}"
  fi
  warn "工作区有未提交改动（不影响镜像内容；镜像严格等于 HEAD=${GIT_HEAD}）："
  printf '%s\n' "${DIRTY}" | head -10 | sed 's/^/[build][WARN]   /' >&2
fi
log "镜像=${IMAGE}  branch=${GIT_BRANCH}  HEAD=${GIT_HEAD}  built_at=${BUILD_TIME}"

# -----------------------------------------------------------------------------
# 1) 出包前断言：HEAD 里的 webviz 前端必须齐全（缺了就没必要浪费一次 build）
# -----------------------------------------------------------------------------
log "断言 HEAD(${GIT_HEAD}) 的 webviz 前端资产 ..."
for f in ${FRONTEND_FILES}; do
  git -C "${SRC_REPO}" cat-file -e "HEAD:${STATIC_REL}/${f}" 2>/dev/null \
    || die "HEAD 缺 webviz 前端资产: ${STATIC_REL}/${f}（charts 模块未提交？）"
done
IDX="$(git -C "${SRC_REPO}" show "HEAD:${STATIC_REL}/index.html")"
for pat in 'charts\.js' 'charts\.css' 'metricsViz' 'eventsViz'; do
  printf '%s' "${IDX}" | grep -q "${pat}" || die "HEAD 的 index.html 缺 ${pat}"
done
log "前端断言 OK（${FRONTEND_FILES// /, } 齐全且 index.html 已挂载 charts）"

# -----------------------------------------------------------------------------
# 2) 同步 docker/ 交付文件到构建上下文（避免上下文与仓库各留一份、互相漂移）
# -----------------------------------------------------------------------------
for f in Dockerfile .dockerignore requirements-app.txt run_app.sh; do
  [ -f "${SRC_REPO}/docker/${f}" ] || die "仓库缺 docker/${f}"
  cp -p "${SRC_REPO}/docker/${f}" "${BUILD_CTX}/${f}"
done
log "已同步 Dockerfile/.dockerignore/requirements-app.txt/run_app.sh 到 ${BUILD_CTX}"

# -----------------------------------------------------------------------------
# 3) 重建 payload/parksim（git archive HEAD，保证是"已提交状态"，无临时文件）
# -----------------------------------------------------------------------------
log "重建 payload/parksim（git archive HEAD）..."
rm -rf "${PAYLOAD_ROOT}/parksim"
mkdir -p "${PAYLOAD_ROOT}/parksim"
git -C "${SRC_REPO}" archive --format=tar HEAD | tar -x -C "${PAYLOAD_ROOT}/parksim"

P="${PAYLOAD_ROOT}/parksim"

# 历史备份红线：ament_python_install_package 不排除 *.bak，备份会被原样带进镜像
NB="$(find "${P}" -type f \( -name '*bak*' -o -name '*.orig' -o -name '*.old' -o -name '*.rej' -o -name '*~' \) 2>/dev/null | wc -l)"
if [ "${NB}" -ne 0 ]; then
  find "${P}" -type f \( -name '*bak*' -o -name '*.orig' -o -name '*.old' -o -name '*.rej' -o -name '*~' \) 2>/dev/null | head -20 >&2
  die "payload/parksim 里仍有 ${NB} 个历史备份文件，拒绝构建"
fi
find "${P}" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true
find "${P}" -type f -name '*.pyc' -delete 2>/dev/null || true
log "payload/parksim OK（0 个历史备份）"

# install 树：colcon 产物不在 git 里，从活源码树 cp（供镜像启动自检用；
# 镜像内第 4 步仍会重新 colcon build，这里是兜底与本地排障用）
if [ -d "${SRC_REPO}/workspace/install" ] && [ ! -d "${P}/workspace/install" ]; then
  cp -a "${SRC_REPO}/workspace/install" "${P}/workspace/install"
  log "已补 payload/parksim/workspace/install"
fi

# -----------------------------------------------------------------------------
# 4) 其余 payload 组件（缺则补；--refresh-payload 时强制重铺）
# -----------------------------------------------------------------------------
ensure() {  # ensure <目标目录> <描述>
  if [ ! -d "$1" ] || [ "${REFRESH_PAYLOAD}" = "1" ]; then return 0; else return 1; fi
}

if ensure "${PAYLOAD_ROOT}/foxy" "foxy"; then
  log "铺 payload/foxy（278M，耗时较长）..."
  rm -rf "${PAYLOAD_ROOT}/foxy"; mkdir -p "${PAYLOAD_ROOT}/foxy"
  cp -a "${FOXY}/." "${PAYLOAD_ROOT}/foxy/"
  find "${PAYLOAD_ROOT}/foxy" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true
  find "${PAYLOAD_ROOT}/foxy" -type f -name '*.pyc' -delete 2>/dev/null || true
else
  log "payload/foxy 已存在，跳过（--refresh-payload 可强制重铺）"
fi

if ensure "${PAYLOAD_ROOT}/dlp" "dlp"; then
  log "铺 payload/dlp ..."
  rm -rf "${PAYLOAD_ROOT}/dlp"; mkdir -p "${PAYLOAD_ROOT}/dlp"
  cp -a "${DLP}/dlp" "${PAYLOAD_ROOT}/dlp/dlp"
  find "${PAYLOAD_ROOT}/dlp" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true
else
  log "payload/dlp 已存在，跳过"
fi

if ensure "${PAYLOAD_ROOT}/dji_subset" "dji"; then
  log "铺 payload/dji_subset（scene/agents/obstacles 真件 + frames/instances 占位）..."
  mkdir -p "${PAYLOAD_ROOT}/dji_subset"
  for n in scene agents obstacles; do
    cp -p "${SRC_REPO}/python/parksim/priorFiles/data/DJI_0012_${n}.json" "${PAYLOAD_ROOT}/dji_subset/" \
      || die "复制 DJI_0012_${n}.json 失败"
  done
  printf '{}' > "${PAYLOAD_ROOT}/dji_subset/DJI_0012_frames.json"
  printf '{}' > "${PAYLOAD_ROOT}/dji_subset/DJI_0012_instances.json"
else
  log "payload/dji_subset 已存在，跳过"
fi

if ensure "${PAYLOAD_ROOT}/priorfiles_root" "priorfiles"; then
  log "铺 payload/priorfiles_root（4 个运行必需 pickle）..."
  mkdir -p "${PAYLOAD_ROOT}/priorfiles_root"
  for f in parking_maneuvers.pickle spots_data.pickle waypoints_graph.pickle agents_data_0012.pickle; do
    cp -p "${SRC_REPO}/python/parksim/priorFiles/${f}" "${PAYLOAD_ROOT}/priorfiles_root/" \
      || die "复制 priorFiles/${f} 失败"
  done
else
  log "payload/priorfiles_root 已存在，跳过"
fi

# 兜底 COPY 的 4 个 pickle 必须与 archive 同源，否则第 6 步 COPY 会用旧件覆盖新件
log "校验 payload/priorfiles_root 与 archive 是否字节一致 ..."
for f in parking_maneuvers.pickle spots_data.pickle waypoints_graph.pickle agents_data_0012.pickle; do
  a="$(sha256sum "${PAYLOAD_ROOT}/parksim/python/parksim/priorFiles/${f}" 2>/dev/null | cut -d' ' -f1)"
  b="$(sha256sum "${PAYLOAD_ROOT}/priorfiles_root/${f}" 2>/dev/null | cut -d' ' -f1)"
  [ -n "${a}" ] && [ "${a}" = "${b}" ] || die "priorfiles_root/${f} 与 archive 不一致（会用旧件覆盖新件），请 --refresh-payload 重铺"
done
log "priorfiles_root 与 archive 一致 OK"

# -----------------------------------------------------------------------------
# 5) 构建
# -----------------------------------------------------------------------------
log "docker build ..."
docker build \
  --build-arg "PARKSIM_GIT_HEAD=${GIT_HEAD}" \
  --build-arg "PARKSIM_GIT_BRANCH=${GIT_BRANCH}" \
  --build-arg "PARKSIM_BUILD_TIME=${BUILD_TIME}" \
  -t "${IMAGE}" \
  "${BUILD_CTX}"

# -----------------------------------------------------------------------------
# 6) 镜像内自检（构建期断言之外的独立复核）
# -----------------------------------------------------------------------------
log "镜像内自检 ..."
docker run --rm --entrypoint bash "${IMAGE}" -c '
  set -e
  S=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim/python/parksim/webviz/static
  for f in index.html app.js style.css charts.js charts.css; do
    test -f "$S/$f" || { echo "MISSING $f"; exit 1; }
  done
  grep -q "charts.js" "$S/index.html"
  grep -q "metricsViz" "$S/index.html"
  echo "--- BUILDINFO ---"
  cat /opt/parksim/BUILDINFO
  echo "--- static ---"
  ls -l "$S"
'

# -----------------------------------------------------------------------------
# 7) 导出 tar.gz + sha256（旧包先留快照，避免新包不可用时无路可退）
# -----------------------------------------------------------------------------
if [ "${DO_SAVE}" = "1" ]; then
  mkdir -p "${OUT_DIR}"
  if [ -f "${TARBALL}" ]; then
    SNAP="${TARBALL}.snapshot-$(date +%Y%m%d_%H%M%S)"
    log "旧包留快照 -> ${SNAP}"
    cp -p "${TARBALL}" "${SNAP}"
  fi
  TMP="${TARBALL}.tmp.$$"
  log "docker save | 压缩 -> ${TARBALL}（1.28G，需数分钟）..."
  if command -v pigz >/dev/null 2>&1; then
    docker save "${IMAGE}" | pigz -6 -p "$(nproc)" > "${TMP}"
  else
    docker save "${IMAGE}" | gzip -6 > "${TMP}"
  fi
  mv "${TMP}" "${TARBALL}"
  ( cd "$(dirname "${TARBALL}")" && sha256sum "$(basename "${TARBALL}")" > "$(basename "${TARBALL}").sha256" )
  log "sha256 -> ${TARBALL}.sha256"
  log "$(cat "${TARBALL}.sha256")"
  log "$(ls -lh "${TARBALL}" | awk '{print "size: "$5}')"
else
  log "按 --no-save 跳过导出 tar.gz"
fi

# -----------------------------------------------------------------------------
# 8) 回收 dangling 镜像（build 每次都会留下 <none>，1.28G 一个）
# -----------------------------------------------------------------------------
if [ "${DO_PRUNE}" = "1" ]; then
  log "回收 dangling 镜像 ..."
  docker image prune -f >/dev/null 2>&1 || true
  docker system df 2>/dev/null | sed 's/^/[build] /'
fi

# -----------------------------------------------------------------------------
# 9) 摘要
# -----------------------------------------------------------------------------
cat <<EOF

$(printf '\033[1;32m')==== 构建完成 ====$(printf '\033[0m')
  镜像    : ${IMAGE}
  源码    : ${GIT_BRANCH} @ ${GIT_HEAD}   (built_at ${BUILD_TIME})
  上下文  : ${BUILD_CTX}
  交付包  : ${TARBALL}$( [ "${DO_SAVE}" = "1" ] && echo " + .sha256" )
  查看版本: docker run --rm --entrypoint cat ${IMAGE} /opt/parksim/BUILDINFO
  同步交付目录脚本: 把 ${SRC_REPO}/docker/{install.sh,run_app.sh,docker-compose.yml,README.md} 覆盖到 ${OUT_DIR}/

EOF
