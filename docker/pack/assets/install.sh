#!/usr/bin/env bash
# =============================================================================
# install.sh —— ParkSim-JTH 便携封装包安装脚本
#
# 适用：Ubuntu 20.04 物理机 / 虚拟机，或 Windows 的 WSL2（Ubuntu 20.04）
#
# 用法：
#   tar -xzf parksim-jth-portable-<date>.tar.gz
#   cd parksim-jth-portable-<date>
#   sudo bash install.sh                 # 推荐：装到默认根
#   bash install.sh --root /data/parksim # 换安装根（会做一次路径改写，见下）
#   bash install.sh --no-apt --no-pip    # 只落盘、不装依赖
#
# 可选参数：
#   --root DIR      安装根（默认 /media/step/data/Yccc7/ParkSim-JTH）
#   --index URL     pip 源（默认 https://pypi.tuna.tsinghua.edu.cn/simple）
#   --no-apt        跳过 apt 安装系统包
#   --no-pip        跳过 pip 安装 Python 包
#   --no-sudo       不用 sudo（当前用户必须能写安装根；pip 自动退化为 --user）
#   -h|--help       帮助
#
# 关于安装根：
#   包里**预置了整套 ROS 2 Foxy**（deps/ros/foxy，278M），它的 setup.bash /
#   ament index 里写死了编译期的绝对路径 /media/step/data/Yccc7/ParkSim-JTH。
#   所以默认就是这个路径——成本为零（只是建目录），且 100% 与构建机同构。
#   若用 --root 换到别的路径，本脚本会把该绝对路径在所有文本文件里改写一遍
#   （best-effort；二进制不动）。这是**未经长期验证**的路径，出问题请换回默认根。
# =============================================================================
set -eo pipefail

CANONICAL_ROOT="/media/step/data/Yccc7/ParkSim-JTH"
INSTALL_ROOT="${PARKSIM_INSTALL_ROOT:-${CANONICAL_ROOT}}"
PIP_INDEX="${PIP_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}"
DO_APT=1
DO_PIP=1
USE_SUDO=1

while [ $# -gt 0 ]; do
  case "$1" in
    --root)   INSTALL_ROOT="$2"; shift 2 ;;
    --root=*) INSTALL_ROOT="${1#--root=}"; shift ;;
    --index)  PIP_INDEX="$2"; shift 2 ;;
    --index=*) PIP_INDEX="${1#--index=}"; shift ;;
    --no-apt) DO_APT=0; shift ;;
    --no-pip) DO_PIP=0; shift ;;
    --no-sudo) USE_SUDO=0; shift ;;
    -h|--help) sed -n '2,40p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "未知参数: $1（用 --help 看用法）" >&2; exit 2 ;;
  esac
done

# 本脚本所在目录就是解出来的包目录
SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log()  { printf '\033[1;34m[install]\033[0m %s\n' "$*"; }
warn() { printf '\033[1;33m[install][WARN]\033[0m %s\n' "$*" >&2; }
die()  { printf '\033[1;31m[install][FATAL]\033[0m %s\n' "$*" >&2; exit 1; }

SUDO=""
if [ "${USE_SUDO}" = "1" ] && [ "$(id -u)" != "0" ] && command -v sudo >/dev/null 2>&1; then
  SUDO="sudo"
fi

# 需要提权时才真用 sudo：能直接写就别去要密码（sudo 在无 TTY 时会直接失败）
run_root() {
  if [ "$(id -u)" = "0" ]; then
    "$@"
  elif [ -n "${SUDO}" ]; then
    ${SUDO} "$@"
  else
    "$@"
  fi
}

# 能不提权就不提权
ensure_dir() {
  mkdir -p "$1" 2>/dev/null || run_root mkdir -p "$1"
}

copy_into() {           # copy_into <src> <dst>
  if [ -w "$(dirname "$2")" ]; then
    cp -a "$1" "$2"
  else
    run_root cp -a "$1" "$2"
  fi
}

# -----------------------------------------------------------------------------
# 0) 环境体检
# -----------------------------------------------------------------------------
log "==== ParkSim-JTH 便携封装包 · 安装 ===="
if [ -r /proc/version ] && grep -qi microsoft /proc/version 2>/dev/null; then
  IN_WSL=1
  log "环境：WSL$(grep -oi 'wsl2\|microsoft' /proc/version | head -1)（检测到 Microsoft 内核）"
else
  IN_WSL=0
  if [ -r /etc/os-release ]; then
    # shellcheck disable=SC1091
    . /etc/os-release
    log "环境：${PRETTY_NAME:-${NAME:-unknown}}"
    case "${VERSION_ID:-}" in
      20.04|"20.04"*) : ;;
      *) warn "本包预置的 ROS 2 Foxy / colcon 产物是在 Ubuntu 20.04 上构建的；当前 ${VERSION_ID:-?} 可能缺 libpython3.8 / libssl1.1，若启动报错请换 Ubuntu 20.04 或 WSL2 的 Ubuntu 20.04。" ;;
    esac
  fi
fi

for d in _ParkSim deps; do
  [ -d "${SRC_DIR}/${d}" ] || die "包内容不完整，缺 ${d}/ —— 请确认在解出来的包目录里执行本脚本（当前：${SRC_DIR}）"
done
[ -f "${SRC_DIR}/requirements-app.txt" ] || die "包内容不完整，缺 requirements-app.txt"

# 前端交付内容红线：曾构建出过 webviz/static 只有 app.js/index.html/style.css 的包，
# 装完能跑、页面却是没有图表模块的旧版，只能靠人工比对才发现。在这里硬拦。
STATIC="${SRC_DIR}/_ParkSim/python/parksim/webviz/static"
MISS=""
for f in index.html app.js style.css charts.js charts.css; do
  [ -f "${STATIC}/${f}" ] || MISS="${MISS} ${f}"
done
[ -z "${MISS}" ] || die "前端交付内容不完整，缺:${MISS}（位于 _ParkSim/python/parksim/webviz/static）。该包为旧版内容，装上去页面会缺图表模块；请改用最新构建的包。"
grep -q 'charts\.js' "${STATIC}/index.html" \
  || die "index.html 未引用 charts.js —— 该包为旧版前端，拒绝安装"
grep -q 'metricsViz' "${STATIC}/index.html" \
  || die "index.html 缺 #metricsViz 容器 —— 该包为旧版前端，拒绝安装"
grep -q 'eventsViz' "${STATIC}/index.html" \
  || die "index.html 缺 #eventsViz 容器 —— 该包为旧版前端，拒绝安装"
log "前端交付内容自检 OK（5 件齐全且 index.html 已挂载 charts）"

# 包内版本信息：让「装的是哪一版」在安装现场当场可见，而不是事后猜。
# BUILDINFO.txt 由 build_pack.sh 生成，含 git_head / git_branch / built_at。
if [ -f "${SRC_DIR}/BUILDINFO.txt" ]; then
  log "包内版本信息（BUILDINFO.txt）："
  sed 's/^/        /' "${SRC_DIR}/BUILDINFO.txt"
else
  warn "包内缺 BUILDINFO.txt，无法确定该包对应的源码版本——排障时会缺少关键线索。"
fi

# -----------------------------------------------------------------------------
# 1) 建安装根并落盘
# -----------------------------------------------------------------------------
log "安装根：${INSTALL_ROOT}"
if [ ! -d "${INSTALL_ROOT}" ]; then
  log "创建目录 ${INSTALL_ROOT} ..."
  ensure_dir "${INSTALL_ROOT}" || die "无法创建 ${INSTALL_ROOT}（权限不足？试试 sudo bash install.sh）"
fi
[ -w "${INSTALL_ROOT}" ] || die "对 ${INSTALL_ROOT} 没有写权限（试试 sudo bash install.sh）"

for d in _ParkSim deps; do
  if [ -d "${INSTALL_ROOT}/${d}" ]; then
    warn "${INSTALL_ROOT}/${d} 已存在，先备份为 ${d}.bak_$(date +%Y%m%d_%H%M%S)"
    mv "${INSTALL_ROOT}/${d}" "${INSTALL_ROOT}/${d}.bak_$(date +%Y%m%d_%H%M%S)" 2>/dev/null \
      || run_root mv "${INSTALL_ROOT}/${d}" "${INSTALL_ROOT}/${d}.bak_$(date +%Y%m%d_%H%M%S)"
  fi
  log "复制 ${d}/ → ${INSTALL_ROOT}/${d}/（可能需要一点时间）"
  copy_into "${SRC_DIR}/${d}" "${INSTALL_ROOT}/${d}"
done
for f in requirements-app.txt run.sh README-portable.md BUILDINFO.txt; do
  if [ -f "${SRC_DIR}/${f}" ]; then
    copy_into "${SRC_DIR}/${f}" "${INSTALL_ROOT}/${f}"
  fi
done
[ -f "${INSTALL_ROOT}/run.sh" ] && chmod +x "${INSTALL_ROOT}/run.sh" 2>/dev/null || true

# -----------------------------------------------------------------------------
# 2) 非默认根 → 改写绝对路径（best-effort）
# -----------------------------------------------------------------------------
if [ "${INSTALL_ROOT}" != "${CANONICAL_ROOT}" ]; then
  log "安装根不是默认路径，改写内嵌绝对路径 ${CANONICAL_ROOT} → ${INSTALL_ROOT} ..."
  FILES="$(grep -rlI -- "${CANONICAL_ROOT}" "${INSTALL_ROOT}" 2>/dev/null || true)"
  N=0
  for f in ${FILES}; do
    sed -i "s|${CANONICAL_ROOT}|${INSTALL_ROOT}|g" "${f}" 2>/dev/null \
      || run_root sed -i "s|${CANONICAL_ROOT}|${INSTALL_ROOT}|g" "${f}"
    N=$((N + 1))
  done
  log "已改写 ${N} 个文本文件"
  warn "改写是 best-effort：只动文本、不动二进制。若 ros2 起不来，请改用默认根 ${CANONICAL_ROOT}。"
fi

# -----------------------------------------------------------------------------
# 3) 系统依赖
# -----------------------------------------------------------------------------
if [ "${DO_APT}" = "1" ]; then
  log "检查系统依赖 ..."
  MISSING=""
  command -v python3  >/dev/null 2>&1 || MISSING="${MISSING} python3"
  python3 -m pip --version >/dev/null 2>&1 || MISSING="${MISSING} python3-pip"
  command -v pkill    >/dev/null 2>&1 || MISSING="${MISSING} procps"
  command -v pstree   >/dev/null 2>&1 || MISSING="${MISSING} psmisc"
  if [ -n "${MISSING}" ]; then
    log "需要安装系统包：${MISSING}"
    # Ubuntu 20.04 已 EOL，镜像同步抖动（"Mirror sync in progress"、Hash 不符）很常见，
    # 所以 update / install 都带重试。
    APT_OPTS="-o Acquire::Retries=5 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30"
    APT_UPDATED=0
    for i in 1 2 3; do
      log "apt-get update（第 ${i} 次）..."
      # shellcheck disable=SC2086
      if run_root env DEBIAN_FRONTEND=noninteractive apt-get ${APT_OPTS} update -qq; then
        APT_UPDATED=1; break
      fi
      warn "apt-get update 第 ${i} 次失败，10 秒后重试"
      sleep 10
    done
    [ "${APT_UPDATED}" = "1" ] || warn "apt-get update 三次都失败（Ubuntu 20.04 已 EOL）。若源失效，把 /etc/apt/sources.list 里的 archive.ubuntu.com / security.ubuntu.com 换成 old-releases.ubuntu.com 再重跑本脚本。"
    # libssl-dev / libtinyxml2-dev：foxy 的 fastrtps 导出 target 里写死了
    # /usr/lib/x86_64-linux-gnu/libtinyxml2.so 的绝对路径，缺了链接期/运行期都会炸。
    # build-essential + python3-dev：给 netifaces 等 C 扩展兜底。
    APT_OK=0
    for i in 1 2 3; do
      log "apt-get install（第 ${i} 次）..."
      # shellcheck disable=SC2086
      if run_root env DEBIAN_FRONTEND=noninteractive apt-get ${APT_OPTS} install -y --no-install-recommends \
          python3 python3-dev python3-pip python3-setuptools \
          build-essential \
          libssl-dev libtinyxml2-dev \
          procps psmisc iproute2 curl ca-certificates; then
        APT_OK=1; break
      fi
      warn "apt-get install 第 ${i} 次失败，10 秒后重试"
      sleep 10
    done
    [ "${APT_OK}" = "1" ] || warn "apt-get install 三次都失败，继续（后面会再体检；体检不过就是系统依赖没装上）"
  else
    log "系统依赖已就绪，跳过 apt"
  fi
else
  log "按 --no-apt 跳过系统依赖安装"
fi

command -v python3 >/dev/null 2>&1 || die "python3 不可用；请手动安装后重跑（sudo apt-get install -y python3 python3-pip）"
PYVER="$(python3 -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
log "python3 = $(command -v python3) (${PYVER})"
case "${PYVER}" in
  3.8|3.8.*) : ;;
  *) warn "预置的 colcon 产物是 cp38 的 .so，当前 python3=${PYVER} 会 import 失败；请用 Ubuntu 20.04（python3.8）。" ;;
esac

# -----------------------------------------------------------------------------
# 4) Python 依赖
# -----------------------------------------------------------------------------
if [ "${DO_PIP}" = "1" ]; then
  log "pip 源：${PIP_INDEX}"
  PIP_FLAGS=""
  if [ "$(id -u)" != "0" ]; then
    SP="$(python3 -c 'import site; print(site.getsitepackages()[0])' 2>/dev/null || echo /usr/lib/python3/dist-packages)"
    if [ ! -w "${SP}" ]; then
      warn "当前用户写不了 ${SP}，pip 改用 --user（装到 ~/.local）"
      PIP_FLAGS="--user"
    fi
  fi

  if ! python3 -m pip --version >/dev/null 2>&1; then
    log "python3 -m pip 不可用，尝试 bootstrap ..."
    run_root env DEBIAN_FRONTEND=noninteractive apt-get update -qq >/dev/null 2>&1 || true
    run_root env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends python3-pip \
      || warn "pip bootstrap 失败；可手动执行：curl -sS https://bootstrap.pypa.io/pip/3.8/get-pip.py | python3"
  fi

  log "升级 pip/setuptools/wheel ..."
  python3 -m pip install ${PIP_FLAGS} -i "${PIP_INDEX}" \
      --retries 5 --timeout 60 \
      "pip==24.2" "setuptools==75.1.0" "wheel==0.44.0" \
    || warn "pip 自身升级失败（不致命，继续装业务依赖）"

  log "安装 requirements-app.txt ..."
  python3 -m pip install ${PIP_FLAGS} -i "${PIP_INDEX}" \
      --retries 5 --timeout 60 \
      -r "${INSTALL_ROOT}/requirements-app.txt" \
    || die "Python 依赖安装失败；可换官方源重跑：PIP_INDEX=https://pypi.org/simple bash install.sh --no-apt"
else
  log "按 --no-pip 跳过 Python 依赖安装"
fi

# -----------------------------------------------------------------------------
# 5) 装后自检
# -----------------------------------------------------------------------------
log "装后自检 ..."
set +u
# ament 的 setup 脚本会引用未定义变量，source 期间必须关掉 nounset
# shellcheck disable=SC1090
source "${INSTALL_ROOT}/deps/ros/foxy/setup.bash" >/dev/null 2>&1 \
  || die "source ROS 2 Foxy 失败：${INSTALL_ROOT}/deps/ros/foxy/setup.bash"
# shellcheck disable=SC1090
source "${INSTALL_ROOT}/_ParkSim/workspace/install/setup.bash" >/dev/null 2>&1 \
  || die "source workspace/install 失败"
set -u

log "ROS_DISTRO=${ROS_DISTRO:-?}"
# 注意：foxy 的 ros2 CLI **没有** --version 参数（会报 unrecognized arguments），用 --help 探测
if command -v ros2 >/dev/null 2>&1 && ros2 --help >/dev/null 2>&1; then
  log "ros2 CLI OK: $(command -v ros2)"
else
  warn "ros2 命令不可用（不影响 webviz，只影响 ros2 node list 之类排查）"
fi
[ -n "${ROS_VERSION:-}" ] || warn "ROS_VERSION 未设置"

python3 - <<'PY' || die "Python 自检失败"
import sys
mods = ['numpy', 'scipy', 'matplotlib', 'pandas', 'yaml', 'PIL', 'aiohttp']
bad = []
for m in mods:
    try:
        __import__(m)
    except Exception as exc:  # noqa: BLE001
        bad.append('%s: %s' % (m, exc))
if bad:
    print('[install][FATAL] 缺失/损坏的 Python 依赖:')
    for b in bad:
        print('   ', b)
    sys.exit(1)
print('[install] python deps OK: ' + ', '.join(mods))
PY

export PYTHONPATH="${INSTALL_ROOT}/deps/dlp-dataset:${PYTHONPATH:-}"
python3 - <<'PY' || die "ROS 消息包自检失败（colcon 产物不可用）"
from parksim.msg import VehicleStateMsg, VehicleInfoMsg  # noqa: F401
from parksim.srv import OccupancySrv  # noqa: F401
import dlp  # noqa: F401
print('[install] parksim.msg/parksim.srv importable; dlp at %s' % dlp.__file__)
PY

# -----------------------------------------------------------------------------
# 5b) 启动前预检 + 旧安装根清理提示
# -----------------------------------------------------------------------------
# 端口预检：run.sh 绑 0.0.0.0:8099，被占用时失败信息不直观，这里提前告诉用户
RUNTIME_PORT="${PORT:-8099}"
PORT_BUSY=0
if command -v ss >/dev/null 2>&1; then
  ss -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${RUNTIME_PORT}\$" && PORT_BUSY=1
elif command -v netstat >/dev/null 2>&1; then
  netstat -ltn 2>/dev/null | awk '{print $4}' | grep -qE "[:.]${RUNTIME_PORT}\$" && PORT_BUSY=1
fi
if [ "${PORT_BUSY}" = "1" ]; then
  warn "端口 ${RUNTIME_PORT} 已被占用；启动时请换端口：PORT=8090 bash ${INSTALL_ROOT}/run.sh"
else
  log "端口 ${RUNTIME_PORT} 空闲，可直接 bash ${INSTALL_ROOT}/run.sh"
fi

# 旧的安装根：反复安装会把上一份改名成 _ParkSim.bak_<时间戳> / deps.bak_<时间戳>。
# 这些旧根里是旧前端（可能没有 charts），既占空间又容易被误跑，显式提示清理。
OLD_ROOTS="$(cd "${INSTALL_ROOT}" 2>/dev/null && ls -d _ParkSim.bak_* deps.bak_* 2>/dev/null || true)"
if [ -n "${OLD_ROOTS}" ]; then
  warn "发现上一次安装留下的旧根（含旧前端，建议确认无用后删除）："
  for d in ${OLD_ROOTS}; do
    printf '          %s/%s\n' "${INSTALL_ROOT}" "${d}" >&2
  done
  warn "确认后清理： rm -rf ${INSTALL_ROOT}/_ParkSim.bak_* ${INSTALL_ROOT}/deps.bak_*"
fi

# -----------------------------------------------------------------------------
# 6) 下一步
# -----------------------------------------------------------------------------
cat <<EOF

$(printf '\033[1;32m')==== 安装完成 ====$(printf '\033[0m')

  安装根 : ${INSTALL_ROOT}
  启动   : bash ${INSTALL_ROOT}/run.sh
  版本   : $( [ -f "${SRC_DIR}/BUILDINFO.txt" ] && (grep -E '^git_(branch|head)|^built_at' "${SRC_DIR}/BUILDINFO.txt" | tr '\n' ' ') || echo '未知（缺 BUILDINFO.txt）' )

EOF

if [ "${IN_WSL}" = "1" ]; then
  cat <<'EOF'
  你是在 WSL2 里：
    · 服务已绑定 0.0.0.0:8099，Windows 侧浏览器直接开 http://127.0.0.1:8099/
    · 也可以回到 Windows，双击包里的 ParkSim.html（或 ParkSim.bat）自动拉起并开页
    · 若 127.0.0.1 不通，取 WSL 的 IP：`ip -4 addr show eth0`，用 http://<那个IP>:8099/

EOF
else
  cat <<'EOF'
  浏览器打开 http://<本机IP>:8099/ （服务绑 0.0.0.0）
  本机直接开：http://127.0.0.1:8099/

EOF
fi

cat <<'EOF'
  常见问题与已知坑请读：README-portable.md
EOF
