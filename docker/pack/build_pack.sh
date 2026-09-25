#!/usr/bin/env bash
# =============================================================================
# build_pack.sh —— 构建 ParkSim-JTH「完整代码封装包」（便携 tar.gz）
#
# 在构建机（STEP-1080）上运行；产物落在 /media/step/data/ParkSim-JTH-portable/
#
# 用法：
#   bash build_pack.sh [日期戳]        # 默认日期戳 = 今天 yyyymmdd
#
# 说明：
#   * 源码取自 _ParkSim 仓库 HEAD（git archive），保证是**已提交**状态，
#     不会把本地未提交的临时文件混进来。
#   * 之后按白名单剪掉运行不需要的大件（7.5G DJI 数据集 / 68M CSV /
#     17M carla / 备份文件 / 日志）。
#   * 再补上运行时必需、但不在 git 里的产物：
#       workspace/install（colcon 产物，含 11 个 Linux .so）
#       deps/ros/foxy（内置 ROS 2 Foxy 树，278M）
#       deps/dlp-dataset/dlp（132K 纯 Python）
#       priorFiles/data/DJI_0012_*（webviz 启动必读；frames/instances 用 {} 占位）
# =============================================================================
set -eo pipefail

DATE="${1:-$(date +%Y%m%d)}"

ENV_ROOT="/media/step/data/Yccc7/ParkSim-JTH"     # 构建机上的环境根
SRC_REPO="${ENV_ROOT}/_ParkSim"                   # 源码仓库
FOXY="${ENV_ROOT}/deps/ros/foxy"                  # 内置 ROS 2 Foxy 树
DLP="${ENV_ROOT}/deps/dlp-dataset"                # DLP 数据集仓库
OUT_DIR="/media/step/data/ParkSim-JTH-portable"   # 交付目录
BUILD="${OUT_DIR}/build"
PKG="parksim-jth-portable-${DATE}"
STAGE="${BUILD}/${PKG}"
TARBALL="${OUT_DIR}/${PKG}.tar.gz"

CANONICAL_ROOT="/media/step/data/Yccc7/ParkSim-JTH"
GIT_HEAD="$(git -C "${SRC_REPO}" rev-parse --short HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git -C "${SRC_REPO}" rev-parse --abbrev-ref HEAD 2>/dev/null || echo unknown)"

log() { printf '[build %s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '[build] FATAL %s\n' "$*" >&2; exit 1; }

[ -d "${SRC_REPO}" ] || die "源码仓库不存在: ${SRC_REPO}"
[ -d "${FOXY}" ]     || die "ROS 2 Foxy 树不存在: ${FOXY}"
[ -d "${DLP}/dlp" ]  || die "dlp 包不存在: ${DLP}/dlp"

log "HEAD=${GIT_HEAD} branch=${GIT_BRANCH} date=${DATE}"

# -----------------------------------------------------------------------------
# 0) 清理并建目录
# -----------------------------------------------------------------------------
rm -rf "${STAGE}"
mkdir -p "${STAGE}" "${OUT_DIR}"
log "staging -> ${STAGE}"

# -----------------------------------------------------------------------------
# 1) 源码：git archive HEAD（已提交状态）
# -----------------------------------------------------------------------------
log "git archive HEAD (${GIT_HEAD}) ..."
mkdir -p "${STAGE}/_ParkSim"
git -C "${SRC_REPO}" archive --format=tar HEAD | tar -x -C "${STAGE}/_ParkSim"

# -----------------------------------------------------------------------------
# 2) 白名单剪枝：去掉运行不需要的大件
# -----------------------------------------------------------------------------
log "pruning non-runtime payload ..."
P="${STAGE}/_ParkSim"

# ---- 出包硬断言：历史备份必须为 0（源码保护红线）------------------------
# 命名形态：*.py.bak_<日期>_<标签> / *_bak_*.py / *.pickle.bak* / *-bak-before-*
# ament_python_install_package 只排除 *.pyc 与 __pycache__，不排除备份，
# 所以 colcon build 会把备份原样复制进 install，必须在这里堵死。
assert_no_backup() {
  local label="$1"
  local root="$2"
  local n
  n=$(find "${root}" -type f \( -name '*bak*' -o -name '*.orig' -o -name '*.old' -o -name '*.rej' -o -name '*~' \) 2>/dev/null | wc -l)
  if [ "${n}" -ne 0 ]; then
    echo "FATAL 出包断言失败：${label} 仍有 ${n} 个历史备份文件，拒绝出包" >&2
    find "${root}" -type f \( -name '*bak*' -o -name '*.orig' -o -name '*.old' -o -name '*.rej' -o -name '*~' \) 2>/dev/null | head -20 >&2
    exit 1
  fi
  log "assert ${label}: 0 个历史备份 OK"
}
rm -rf \
  "${P}/carla_PythonAPI" \
  "${P}/docs" \
  "${P}/vehicle_log" \
  "${P}/.vscode" \
  "${P}/workspace/build" \
  "${P}/workspace/log" \
  "${P}/python/parksim/trajectory_predict" \
  "${P}/python/parksim/priorFiles/data" \
  "${P}/python/parksim/priorFiles/_archive_jth_old_20260915" \
  "${P}/python/parksim/webviz_backup_20260912_150903" \
  "${P}/python/parksim/webviz/run_web_sim.sh" \
  "${P}/tools/_archive" \
  # NOTE: 历史备份不再逐条枚举，统一由下方 "通用历史备份清扫" 处理
  #       （逐条枚举漏掉了 *.py.bak_<日期>_<标签> 形态的 15 份 stanley 备份）
  2>/dev/null || true
rm -f "${P}"/*.log 2>/dev/null || true
# 所有 __pycache__ / .pyc
find "${P}" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true
find "${P}" -type f -name '*.pyc' -delete 2>/dev/null || true

# ---- 通用历史备份清扫（不依赖具体命名）----------------------------------
log "sweeping history backups (*bak* / *.orig / *.old / *.rej / *~) ..."
find "${P}" -type f \( -name '*bak*' -o -name '*.orig' -o -name '*.old' -o -name '*.rej' -o -name '*~' \) -print -delete 2>/dev/null | head -40 | sed 's/^/  removed: /'
find "${P}" -type d -name '*bak*' -prune -exec rm -rf {} + 2>/dev/null || true
find "${P}" -type f -name '*.log' -delete 2>/dev/null || true
assert_no_backup "源码树(source)" "${P}"

# 剪枝后不得再引用被删掉的备份文件（防止有人把 .bak 写进配置）
if grep -rIl '\.bak_\|_archive_jth_old' "${P}/python/parksim" "${P}/workspace" 2>/dev/null | grep -v '\.md$' | head -5 | grep -q .; then
  log "WARN 仍有文件引用 .bak_ / _archive_jth_old（仅提示）："
  grep -rIl '\.bak_\|_archive_jth_old' "${P}/python/parksim" "${P}/workspace" 2>/dev/null | grep -v '\.md$' | head -5 || true
fi

# -----------------------------------------------------------------------------
# 3) 补：colcon 产物 workspace/install（不在 git 里）
# -----------------------------------------------------------------------------
log "copying workspace/install (colcon artifacts) ..."
rm -rf "${P}/workspace/install"
cp -a "${SRC_REPO}/workspace/install" "${P}/workspace/install"
find "${P}/workspace/install" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true

# install 树是从活源码树 cp -a 来的，ament 不排除备份 -> 必须单独清扫
log "sweeping history backups in install tree ..."
find "${P}/workspace/install" -type f \( -name '*bak*' -o -name '*.orig' -o -name '*.old' -o -name '*.rej' -o -name '*~' \) -print -delete 2>/dev/null | head -40 | sed 's/^/  removed: /'
assert_no_backup "install树" "${P}/workspace/install"

# -----------------------------------------------------------------------------
# 4) 补：内置 ROS 2 Foxy 树 + dlp 包
# -----------------------------------------------------------------------------
log "copying bundled ROS 2 Foxy tree (may take a while) ..."
mkdir -p "${STAGE}/deps/ros"
cp -a "${FOXY}" "${STAGE}/deps/ros/foxy"
find "${STAGE}/deps/ros/foxy" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true
find "${STAGE}/deps/ros/foxy" -type f -name '*.pyc' -delete 2>/dev/null || true

log "copying dlp python package ..."
mkdir -p "${STAGE}/deps/dlp-dataset"
cp -a "${DLP}/dlp" "${STAGE}/deps/dlp-dataset/dlp"
find "${STAGE}/deps/dlp-dataset" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true

# -----------------------------------------------------------------------------
# 5) 补：webviz 启动必读的 DJI_0012 最小子集
#    scene/agents/obstacles 用真实文件（120KB）；
#    frames/instances 是 269MB 大件，jth_b1 静态图层走 layout_rotated 分支、
#    内容不被使用，但文件必须存在，用 `{}` 占位。
# -----------------------------------------------------------------------------
log "adding DJI_0012 minimal subset ..."
DATA="${P}/python/parksim/priorFiles/data"
mkdir -p "${DATA}"
for n in scene agents obstacles; do
  cp -a "${SRC_REPO}/python/parksim/priorFiles/data/DJI_0012_${n}.json" "${DATA}/"
done
printf '{}' > "${DATA}/DJI_0012_frames.json"
printf '{}' > "${DATA}/DJI_0012_instances.json"

# -----------------------------------------------------------------------------
# 6) 资产断言（缺任何一个都让构建失败，避免到目标机才炸）
# -----------------------------------------------------------------------------
log "asserting runtime assets ..."
PF="${P}/python/parksim/priorFiles"
for f in parking_maneuvers.pickle spots_data.pickle waypoints_graph.pickle agents_data_0012.pickle; do
  [ -f "${PF}/${f}" ] || die "缺少 priorFiles 资产: ${f}"
done
for f in DJI_0012_scene.json DJI_0012_agents.json DJI_0012_obstacles.json DJI_0012_frames.json DJI_0012_instances.json; do
  [ -f "${PF}/data/${f}" ] || die "缺少 DLP 文件: ${f}"
done
for f in map.yaml layout_rotated.json spots_data.pickle waypoints_graph.pickle \
         parking_maneuvers_per_spot.pickle base_map_clean.png obstacles.json zone_overview.json; do
  [ -f "${PF}/maps/jth_b1/${f}" ] || die "缺少 jth_b1 资产: ${f}"
done
[ -f "${STAGE}/deps/dlp-dataset/dlp/base_map.png" ] || die "缺少 dlp/base_map.png（/base_map.png 路由依赖）"
PF="${P}/python/parksim/priorFiles"
export PF
python3 - <<'PY'
import json, os
base = os.environ['PF'] + '/data/DJI_0012_%s.json'
for n in ('scene', 'agents', 'obstacles', 'frames', 'instances'):
    with open(base % n) as fh:
        json.load(fh)
print('[build] DJI_0012 json all valid')
PY
log "asset assertions OK"

# -----------------------------------------------------------------------------
# 6b) 前端交付内容断言
#     曾构建出过 webviz/static 只有 app.js/index.html/style.css 的包：包能装能跑，
#     页面却是没有图表模块的旧版，只能靠人工比对才发现。出包前硬拦。
# -----------------------------------------------------------------------------
log "asserting webviz frontend assets ..."
ST="${P}/python/parksim/webviz/static"
for f in index.html app.js style.css charts.js charts.css; do
  [ -f "${ST}/${f}" ] || die "缺少 webviz 前端资产: webviz/static/${f}（charts 模块未提交？）"
done
grep -q 'charts\.js'  "${ST}/index.html" || die "index.html 未引用 charts.js"
grep -q 'charts\.css' "${ST}/index.html" || die "index.html 未引用 charts.css"
grep -q 'metricsViz'   "${ST}/index.html" || die "index.html 缺 #metricsViz 容器"
grep -q 'eventsViz'    "${ST}/index.html" || die "index.html 缺 #eventsViz 容器"
log "webviz 前端断言 OK（5 件齐全且已挂载 charts）"

# -----------------------------------------------------------------------------
# 7) 脚本与文档（由 build_pack.sh 同目录的 assets/ 提供；若不存在则跳过）
# -----------------------------------------------------------------------------
ASSETS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/assets"
if [ -d "${ASSETS}" ]; then
  log "copying scripts & docs from ${ASSETS} ..."
  cp -a "${ASSETS}/." "${STAGE}/"
  cp -a "${SRC_REPO}/docker/requirements-app.txt" "${STAGE}/requirements-app.txt"
  chmod +x "${STAGE}/install.sh" "${STAGE}/run.sh"
else
  log "WARN 未找到 assets/ 目录，脚本与文档不会入包"
fi

# 写一份包内元信息，便于目标机自检
cat > "${STAGE}/BUILDINFO.txt" <<EOF
package      : ${PKG}
built_at     : $(date '+%Y-%m-%d %H:%M:%S %z')
built_on     : $(hostname)
git_repo     : _ParkSim
git_branch   : ${GIT_BRANCH}
git_head     : ${GIT_HEAD}
canonical_root: ${CANONICAL_ROOT}
payload      : _ParkSim/ (源码白名单) + workspace/install + deps/ros/foxy + deps/dlp-dataset/dlp
webviz_frontend: $(cd "${P}/python/parksim/webviz/static" && sha256sum index.html app.js style.css charts.js charts.css 2>/dev/null | awk '{printf "%s=%s ", $2, substr($1,1,12)}')
EOF

# -----------------------------------------------------------------------------
# 8) 打包
# -----------------------------------------------------------------------------
log "packing ${TARBALL} ..."
rm -f "${TARBALL}"
if command -v pigz >/dev/null 2>&1; then
  tar -cf - -C "${BUILD}" "${PKG}" | pigz -9 -p "$(nproc)" > "${TARBALL}"
else
  tar -czf "${TARBALL}" -C "${BUILD}" "${PKG}"
fi
log "done: ${TARBALL}"
ls -lh "${TARBALL}"

# 完整性指纹：376MB/55MB 的包在拷贝、U 盘、跨网络传输中损坏并不罕见，
# 有了 .sha256，目标机 install.sh 才能校验而不是"装上才发现坏"。
if command -v sha256sum >/dev/null 2>&1; then
  ( cd "$(dirname "${TARBALL}")" && sha256sum "$(basename "${TARBALL}")" > "$(basename "${TARBALL}").sha256" )
  log "sha256 -> ${TARBALL}.sha256"
  log "$(cat "${TARBALL}.sha256")"
fi

cat <<EOF

[build] ==== 出包完成 ====
  源码     : ${GIT_BRANCH} @ ${GIT_HEAD}
  构建时间 : $(date '+%Y-%m-%d %H:%M:%S %z')
  产物     : ${TARBALL}$( [ -f "${TARBALL}.sha256" ] && echo " + .sha256" )
  前端指纹 : $(cd "${P}/python/parksim/webviz/static" && sha256sum charts.js 2>/dev/null | cut -c1-16)  (charts.js 前 16 位)

EOF
