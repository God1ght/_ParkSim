#!/usr/bin/env bash
# =============================================================================
# 运行期闭包断言（parksim-jth 多阶段瘦身版专用）
#
# 为什么需要它：多阶段构建把 apt 层从 403 MB 压到"实测运行期闭包"。
# 「闭包够不够」如果只靠静态推算，就是一句无法证伪的话。本脚本把它变成
# 构建期可失败的断言，并且**带负向对照**——只判"新增缺口"，不掩盖 v1 既有缺口。
#
# 五段：
#   C1a 应用自身 install 树 unresolved 必须 = 0            （严格，无豁免）
#   C1b foxy 全量扫描 unresolved ⊆ v1 冻结 allow-list      （负向对照：只判新增）
#   C1c workspace/build 存在（install 下 typesupport .so 的 RUNPATH 指向它）
#   C2  真实加载两条链：
#         C2a 应用入口链（cwd=$PARKSIM_ROOT/python，sys.path[0]=webviz 目录，exec server.py
#             的模块级代码）—— server.py 模块级只用到 numpy/yaml/aiohttp/PIL/rclpy/
#             std_msgs/rosidl_runtime_py/dlp，并调用 get_message('parksim/msg/...')；
#             **它不导入应用自身的 python/parksim 包，也不改 sys.path**（已用 AST 核实）。
#         C2b ROS 节点链（cwd=/ 中性目录，import parksim.msg/srv 并真实建节点/发布/服务）
#             —— parksim.msg/srv 只被 workspace/src 下的 ROS 节点使用。
#   C3  反向对照：allow-list 里那些缺失库，是否真被这两个进程加载过
#   C4a `ros2 launch --help` 退出码必须 0（launch 入口点可加载）
#   C4b 真实 `ros2 launch parksim parksim.launch.py`，断言 /simulator 出现
#
# 退出码：0 全通过；1 有失败（构建应当中止）。
# =============================================================================
set -uo pipefail

ALLOWLIST="${1:-/opt/parksim-assert/so-allowlist-v1.txt}"
FOXY="${ROS_FOXY_ROOT:?ROS_FOXY_ROOT 未设置}"
PS="${PARKSIM_ROOT:?PARKSIM_ROOT 未设置}"
DLP="${DLP_DATASET_ROOT:-${PARKSIM_ENV_ROOT:-}/deps/dlp-dataset}"

fail=0
W="$(mktemp -d)"
MAPS_ALL="${W}/loaded_so_all.txt"
: > "${MAPS_ALL}"

hdr() { printf '\n===== %s =====\n' "$1"; }
collect_maps() {   # $1 = 某个 python 进程导出的 maps 文件
  [ -s "$1" ] && cat "$1" >> "${MAPS_ALL}"
  return 0
}

# 与 run_app.sh 保持一致：dlp 是 develop 安装，靠 PYTHONPATH 注入
export PYTHONPATH="${DLP}:${PYTHONPATH:-}"

set +u
# shellcheck disable=SC1090
source "${FOXY}/setup.bash" >/dev/null 2>&1 || true
source "${PS}/workspace/install/setup.bash" >/dev/null 2>&1 || true
set -u

echo "ROS_FOXY_ROOT = ${FOXY}"
echo "PARKSIM_ROOT  = ${PS}"
echo "DLP_DATASET_ROOT = ${DLP}"
echo "allow-list    = ${ALLOWLIST}"

# -----------------------------------------------------------------------------
# C1a —— 应用自身的库：零容忍
# -----------------------------------------------------------------------------
hdr "C1a  install 树（应用自身 .so）unresolved 必须 = 0"

find "${PS}/workspace/install" -name '*.so*' -type f 2>/dev/null > "${W}/inst_all.txt"
: > "${W}/inst_gap.txt"
while IFS= read -r f; do
  # 注意：**不要**写成 `ldd "$f" | grep -q 'not found'`。
  # 本脚本开了 set -o pipefail；grep -q 命中即退出会给 ldd 发 SIGPIPE，
  # ldd 以 141 退出后 pipefail 让整个管道判为非零，if 就走了 else 分支
  # —— 结果是「有缺口」被静默判成「无缺口」（假阴性，会漏掉真实的新增缺口）。
  # 用 grep -c 消费全部输出、且把管道状态丢进 $( )，彻底消除这个竞态。
  n=$(ldd "$f" 2>/dev/null | grep -c 'not found')
  if [ "${n:-0}" -gt 0 ]; then printf '%s\n' "$f" >> "${W}/inst_gap.txt"; fi
done < "${W}/inst_all.txt"

n_inst_all=$(wc -l < "${W}/inst_all.txt")
n_inst_gap=$(wc -l < "${W}/inst_gap.txt")
echo "install 树 .so 文件数      = ${n_inst_all}"
echo "install 树 unresolved 个数 = ${n_inst_gap}"

if [ "${n_inst_gap}" -ne 0 ]; then
  echo "[C1a][FAIL] 应用自身库存在未解析依赖："
  while IFS= read -r f; do
    ldd "$f" 2>/dev/null | grep 'not found' | sed "s|^|  $(basename "$f"): |"
  done < "${W}/inst_gap.txt"
  fail=1
else
  echo "[C1a] OK —— 应用自身 ${n_inst_all} 个 .so 依赖全部可解析"
fi

# -----------------------------------------------------------------------------
# C1b —— foxy 全量：只能出现 v1 已知缺口，不得新增
# -----------------------------------------------------------------------------
hdr "C1b  foxy 全量扫描 unresolved ⊆ v1 冻结 allow-list（只判新增）"

if [ ! -f "${ALLOWLIST}" ]; then
  echo "[C1b][FAIL] 找不到 allow-list：${ALLOWLIST}"
  exit 1
fi

find "${FOXY}" -name '*.so*' -type f 2>/dev/null > "${W}/foxy_all.txt"
: > "${W}/foxy_gap_abs.txt"
while IFS= read -r f; do
  # 同 C1a：用 grep -c + $( ) 规避 pipefail 下的 SIGPIPE 假阴性
  n=$(ldd "$f" 2>/dev/null | grep -c 'not found')
  if [ "${n:-0}" -gt 0 ]; then printf '%s\n' "$f" >> "${W}/foxy_gap_abs.txt"; fi
done < "${W}/foxy_all.txt"

sed "s|^${FOXY}/||" "${W}/foxy_gap_abs.txt" | sort -u > "${W}/foxy_gap_rel.txt"
sort -u "${ALLOWLIST}" > "${W}/allow_sorted.txt"
comm -23 "${W}/foxy_gap_rel.txt" "${W}/allow_sorted.txt" > "${W}/newgaps.txt"
comm -13 "${W}/foxy_gap_rel.txt" "${W}/allow_sorted.txt" > "${W}/fixed.txt"

echo "foxy .so 文件数           = $(wc -l < "${W}/foxy_all.txt")"
echo "foxy 当前 unresolved 个数 = $(wc -l < "${W}/foxy_gap_rel.txt")"
echo "v1 冻结 allow-list 行数   = $(wc -l < "${W}/allow_sorted.txt")"
echo "相对 v1 已消失的缺口个数  = $(wc -l < "${W}/fixed.txt")   （信息性，不判失败）"
if [ -s "${W}/fixed.txt" ]; then
  echo "  已消失清单（仅供诊断；出现即说明扫描有非确定性，需查清）："
  sed 's|^|    |' "${W}/fixed.txt"
fi
echo "新增缺口个数              = $(wc -l < "${W}/newgaps.txt")"

if [ -s "${W}/newgaps.txt" ]; then
  echo "[C1b][FAIL] 出现 v1 中不存在的『新』未解析 .so —— 瘦身很可能删掉了运行期要用的 apt 包："
  sed 's|^|  |' "${W}/newgaps.txt"
  while IFS= read -r rel; do
    echo "  --- ${rel}"
    ldd "${FOXY}/${rel}" 2>/dev/null | grep 'not found' | sed 's|^|      |'
  done < "${W}/newgaps.txt"
  fail=1
else
  echo "[C1b] OK —— 无新增缺口；allow-list 内缺口均为 v1 既有（Connext/rviz/Qt/OGRE/OpenCV/assimp/SDL2 等应用不使用项）"
fi

# -----------------------------------------------------------------------------
# C1c —— workspace/build 必须存在：install 下的 typesupport .so 的 RUNPATH 指向它
#   这是本版踩过并记录在案的坑：只 COPY install/ 会出现"import 能过、加载 typesupport 炸"。
#   C1a 的 ldd 其实已能间接覆盖，这里做一条显式断言，把机制写清楚。
# -----------------------------------------------------------------------------
hdr "C1c  workspace/build 存在性（install 的 typesupport RUNPATH 目标）"

if [ ! -d "${PS}/workspace/build" ]; then
  echo "[C1c][FAIL] ${PS}/workspace/build 不存在"
  fail=1
else
  n_ts=$(find "${PS}/workspace/build" -name 'parksim_s__*.so' 2>/dev/null | wc -l)
  echo "workspace/build 下 parksim_s__*.so 个数 = ${n_ts}"
  if [ "${n_ts}" -lt 3 ]; then
    echo "[C1c][FAIL] typesupport .so 少于 3 个，install 的 RUNPATH 会悬空"
    find "${PS}/workspace/build" -name '*.so' 2>/dev/null | head -10 | sed 's|^|  |'
    fail=1
  else
    echo "[C1c] OK —— workspace/build 已随镜像带入"
  fi
fi

if command -v readelf >/dev/null 2>&1; then
  rp_bad=0
  while IFS= read -r f; do
    while IFS= read -r d; do
      [ -n "${d}" ] || continue
      case "${d}" in
        '$ORIGIN'*|'$LIB'*|'$PLATFORM'*) continue ;;
      esac
      if [ ! -d "${d}" ]; then
        echo "  [C1c][FAIL] RUNPATH 目录不存在: ${d}  （来自 $(basename "$f")）"
        rp_bad=1
      fi
    done < <(readelf -d "$f" 2>/dev/null | sed -n 's/.*RUNPATH.*\[\(.*\)\].*/\1/p' | tr ':' '\n')
  done < "${W}/inst_all.txt"
  if [ "${rp_bad}" -eq 0 ]; then
    echo "[C1c] OK —— install 树 .so 的 RUNPATH 目标目录全部存在（readelf 模式）"
  else
    fail=1
  fi
else
  echo "[C1c] 注意：运行期镜像未装 binutils（readelf 不可用），RUNPATH 逐项核对跳过；"
  echo "       该属性由 C1a（ldd 全解析）间接覆盖——RUNPATH 悬空会让 ldd 报 not found。"
fi

# -----------------------------------------------------------------------------
# C2 —— 真实加载（正向对照）
#   测量得来：server.py 的模块级导入面是（见本版报告 §C2）
#     stdlib: argparse asyncio json math os re signal subprocess sys threading time pathlib io
#     第三方: numpy yaml aiohttp PIL
#     ROS   : rclpy rclpy.node std_msgs.msg rosidl_runtime_py.utilities
#     数据集: dlp dlp.dataset dlp.visualizer
#     注意 server.py **不**导入 parksim.msg；ROS 生成的消息包只被 workspace/src 下的
#     ROS 节点（simulator_node / vehicle_node / visualizer_node / test_vehicle_node）使用。
#   所以 C2 必须分两条链，否则要么漏测、要么误测：
#     C2a 应用入口链（cwd = $PARKSIM_ROOT/python，与 run_app.sh 一致）
#     C2b ROS 节点链（cwd = / 中性目录，否则 cwd 里的 python/parksim 会遮蔽
#          install 的 parksim 包 —— 这是本版排查出来的一条既有事实，非瘦身引入）
# -----------------------------------------------------------------------------
hdr "C2a 应用入口链：真实 cwd 下 exec server.py 的模块级导入面 + 应用包"

if ( cd "${PS}/python" && python3 - <<'PY'
import importlib.util, os, sys

# 0) 复刻真实入口的 sys.path[0]
#    run_app.sh:  cd "$PARKSIM_ROOT/python" && exec python3 -u "$PARKSIM_ROOT/python/parksim/webviz/server.py"
#    `python3 -u <script>` 的 sys.path[0] 是**脚本所在目录**，不是 cwd。
#    实测这一步不能省：用 `python3 -`（sys.path[0]=''=cwd）会让 cwd 下的
#    python/parksim 遮蔽 install 的 parksim 包，server.py 第 99 行的
#    get_message('parksim/msg/VehicleStateMsg') 就会 ModuleNotFoundError。
WEBVIZ = os.path.join(os.environ["PARKSIM_ROOT"], "python/parksim/webviz")
sys.path[0] = WEBVIZ
print("cwd =", os.getcwd())
print("sys.path[0] =", sys.path[0], "（复刻 python3 -u server.py）")

# 1) 第三方运行期依赖（server.py 模块级用到）
import numpy, yaml, aiohttp, PIL                     # noqa: E402
print("numpy/yaml/aiohttp/PIL 版本:", numpy.__version__, yaml.__version__,
      aiohttp.__version__, PIL.__version__)

# 2) ROS 侧（server.py 模块级用到）
import rclpy                                          # noqa: E402
import rclpy.node
import std_msgs.msg
import rosidl_runtime_py.utilities
print("rclpy/std_msgs/rosidl_runtime_py 导入 OK")

# 3) dlp 数据集（develop 安装，靠 PYTHONPATH）
import dlp, dlp.dataset, dlp.visualizer               # noqa: E402
print("dlp/dlp.dataset/dlp.visualizer 导入 OK ->", getattr(dlp, "__file__", "?"))

# 4) 真的 exec server.py 的模块级代码。测量得知它只有一处 `if __name__ == "__main__"`
#    守卫，所以以别的模块名导入不会启动服务；模块级第 99/100 行的
#    get_message('parksim/msg/...') 会把 msg 类型支持链真正拉起来。
p = os.path.join(WEBVIZ, "server.py")
spec = importlib.util.spec_from_file_location("_wb_server_import_probe", p)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
print("server.py 模块级导入面全部通过")
print("STATE_MSG =", getattr(mod, "STATE_MSG", "<无此属性>"))
print("INFO_MSG  =", getattr(mod, "INFO_MSG", "<无此属性>"))
print("顶层可调用对象数 =", len([k for k, v in vars(mod).items() if callable(v)]))

# 5) 导出本进程已加载的 .so，供 C3
mapped = set()
with open("/proc/%d/maps" % os.getpid()) as fh:
    for line in fh:
        parts = line.split()
        if len(parts) >= 6 and parts[-1].startswith("/"):
            mapped.add(parts[-1])
with open("/tmp/_loaded_so_c2a.txt", "w") as fh:
    fh.write("\n".join(sorted(mapped)) + "\n")
print("[C2a] OK —— 应用入口链真实可加载；已导出映射条目 %d 条" % len(mapped))
PY
)
then
  echo "[C2a] OK"
  collect_maps /tmp/_loaded_so_c2a.txt
else
  echo "[C2a][FAIL] 应用入口链加载失败（见上方 traceback）"
  fail=1
fi

hdr "C2b ROS 节点链：msg/srv + 真实建节点/发布/服务 + 断言 rmw = fastrtps"

if ( cd / && python3 - <<'PY'
import os, sys
try:
    import rclpy
    from rclpy.node import Node
    from parksim.msg import VehicleStateMsg, VehicleInfoMsg
    from parksim.srv import OccupancySrv
except Exception as exc:                     # noqa: BLE001
    print("[C2b][FAIL] 导入失败: %r" % (exc,))
    sys.exit(1)

rmw = None
try:
    rmw = rclpy.get_rmw_implementation_identifier()
except Exception:
    try:
        from rclpy.utilities import get_rmw_implementation_identifier as _g
        rmw = _g()
    except Exception as exc:                 # noqa: BLE001
        print("[C2b][FAIL] 取不到 rmw 实现标识: %r" % (exc,))
        sys.exit(1)
print("rmw_implementation =", rmw)
if "fastrtps" not in str(rmw):
    print("[C2b][FAIL] rmw 实现不是 fastrtps")
    sys.exit(1)

import parksim.msg as _pm
print("parksim.msg ->", _pm.__file__)

rclpy.init()
try:
    n = Node("assert_runtime_deps")
    n.create_publisher(VehicleStateMsg, "assert_topic", 10)
    def _cb(req, resp):
        return resp
    n.create_service(OccupancySrv, "assert_srv", _cb)
    m = VehicleStateMsg()
    i = VehicleInfoMsg()
    for _ in range(10):
        rclpy.spin_once(n, timeout_sec=0.05)
    # Foxy 的 Node 没有 count_services()，用 get_service_names_and_types()
    svc_names = [s for s, _ in n.get_service_names_and_types()]
    print("publisher_count(assert_topic) =", n.count_publishers("assert_topic"))
    print("服务列表含 assert_srv         =", any("assert_srv" in s for s in svc_names))
    print("VehicleStateMsg 字段数        =", len(m.get_fields_and_field_types()))
    print("VehicleInfoMsg 字段数         =", len(i.get_fields_and_field_types()))
    if not any("assert_srv" in s for s in svc_names):
        print("[C2b][FAIL] 服务 assert_srv 未出现在服务列表:", svc_names[:10])
        sys.exit(1)
    n.destroy_node()
finally:
    rclpy.shutdown()

mapped = set()
with open("/proc/%d/maps" % os.getpid()) as fh:
    for line in fh:
        parts = line.split()
        if len(parts) >= 6 and parts[-1].startswith("/"):
            mapped.add(parts[-1])
with open("/tmp/_loaded_so_c2b.txt", "w") as fh:
    fh.write("\n".join(sorted(mapped)) + "\n")
print("[C2b] OK —— 节点/发布/服务/消息 真实可用；已导出映射条目 %d 条" % len(mapped))
PY
)
then
  echo "[C2b] OK"
  collect_maps /tmp/_loaded_so_c2b.txt
else
  echo "[C2b][FAIL] ROS 节点链加载失败（见上方 traceback）"
  fail=1
fi

# -----------------------------------------------------------------------------
# C3 —— 反向对照：allow-list 内缺失的库，这两个进程到底加载过没有
#   把"这些缺口对本应用不可达"从推断变成检查。若命中，说明某个被加载的库
#   依赖了缺失库，构建应当失败。
# -----------------------------------------------------------------------------
hdr "C3  反向对照：allow-list 内的缺失库是否被应用加载"

: > "${W}/allowlibs.txt"
while IFS= read -r rel; do
  f="${FOXY}/${rel}"
  [ -f "$f" ] || continue
  ldd "$f" 2>/dev/null | grep 'not found' | awk '{print $1}'
done < "${W}/allow_sorted.txt" | sort -u > "${W}/allowlibs.txt"

echo "allow-list 涉及的缺失库名（去重）= $(wc -l < "${W}/allowlibs.txt")"

if [ ! -s "${MAPS_ALL}" ]; then
  echo "[C3][WARN] 没拿到任何加载清单，本段跳过（不判失败，但不得据此声称已验证）"
else
  sort -u "${MAPS_ALL}" > "${W}/loaded_sorted.txt"
  echo "两个进程已加载的绝对路径条目（去重）= $(wc -l < "${W}/loaded_sorted.txt")"
  hit=0
  while IFS= read -r lib; do
    [ -n "${lib}" ] || continue
    if grep -q "/${lib}\$" "${W}/loaded_sorted.txt" 2>/dev/null; then
      echo "  [C3][FAIL] 应用加载了 ${lib}，而它的依赖是缺失的"
      grep "/${lib}\$" "${W}/loaded_sorted.txt" | sed 's|^|      |'
      hit=1
    fi
  done < "${W}/allowlibs.txt"
  if [ "${hit}" -eq 0 ]; then
    echo "[C3] OK —— 应用真实加载清单中命中缺失库 0 个（allow-list 内缺口对本应用不可达）"
  else
    fail=1
  fi
fi

# -----------------------------------------------------------------------------
# C4 —— 真实启动仿真（多阶段专属，2026-09-26 因一次真实回归而新增）
#   背景：v2-thin 首次构建时 C1/C2/C3 全过，但容器里点「开启仿真」直接失败：
#       Failed to load entry point 'launch': No module named 'lark'
#       RuntimeError: simulator exited early (code 1)
#   根因是运行期 pip 子集误删了 lark-parser —— 它是 `ros2 launch` 的**运行期**依赖
#   （launch 前端解析器用 lark），而不是当初以为的「仅构建期」。
#   C2a/C2b 都没调用 ros2 launch，所以整类缺口在断言里是隐形的。
#   教训：断言的覆盖范围必须包含「结论适用的那类场景」。应用启动仿真的真实动作就是
#   `ros2 launch parksim parksim.launch.py`，于是补上这一段。
#     C4a  `ros2 launch --help` 必须退出码 0（入口点可加载，最便宜的探针）
#     C4b  真的 `ros2 launch` 一次，断言 /simulator 出现在节点列表里
# -----------------------------------------------------------------------------
hdr "C4a  ros2 launch 入口点可加载（缺 lark 一类包会在这里直接失败）"

if ros2 launch --help > "${W}/launch_help.txt" 2>&1; then
  echo "[C4a] OK —— ros2 launch 入口点可加载"
else
  echo "[C4a][FAIL] ros2 launch 入口点加载失败："
  grep -iE "no module named|failed to load entry point" "${W}/launch_help.txt" \
    | sort -u | head -5 | sed 's|^|  |'
  fail=1
fi

hdr "C4b  真实启动仿真：ros2 launch parksim parksim.launch.py（隔离 ROS_DOMAIN_ID=96）"

C4B_LOG="${W}/c4b_launch.log"
export ROS_DOMAIN_ID=96
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="${W}/c4b_roslog"      # 别把 launch 日志写进 /root/.ros
export PYTHONDONTWRITEBYTECODE=1          # 别在 install 树里留 __pycache__
touch "${W}/c4b_marker"

( cd "${PS}/python" && timeout -k 5 50 ros2 launch parksim parksim.launch.py gui:=false map:=jth_b1 ) \
  > "${C4B_LOG}" 2>&1 &
C4B_PID=$!
sleep 32
C4B_NODES="$(timeout 15 ros2 node list 2>/dev/null | sort | tr '\n' ' ')"
wait "${C4B_PID}" 2>/dev/null || true

echo "  ros2 node list = ${C4B_NODES:-<空>}"

if printf '%s' "${C4B_NODES}" | grep -q '/simulator'; then
  echo "[C4b] OK —— /simulator 已出现在节点列表（仿真真实起得来）"
else
  echo "[C4b][FAIL] 未见到 /simulator；launch 输出尾部："
  tail -20 "${C4B_LOG}" | sed 's|^|  |'
  fail=1
fi

if grep -qiE "no module named|failed to load entry point" "${C4B_LOG}"; then
  echo "[C4b][FAIL] launch 输出里出现缺模块/入口加载失败："
  grep -iE "no module named|failed to load entry point" "${C4B_LOG}" \
    | sort -u | head -5 | sed 's|^|  |'
  fail=1
else
  echo "[C4b] OK —— launch 输出无缺模块 / 入口加载失败"
fi

if grep -q "A vehicle with id" "${C4B_LOG}"; then
  echo "[C4b] OK —— 观察到发车日志 $(grep -c 'A vehicle with id' "${C4B_LOG}") 条"
else
  echo "[C4b] 注意：50 s 内未观察到发车日志（不判失败；发车间隔取决于默认场景）"
fi

# 清理 C4b 的运行副产物（同一 RUN 层内删除，不进镜像）
command -v pkill >/dev/null 2>&1 && { pkill -f simulator_node.py || true; pkill -f vehicle_node.py || true; }
rm -rf "${ROS_LOG_DIR}" /root/.ros 2>/dev/null || true
find "${PS}/workspace/install" -type f -name '*.pyc' -newer "${W}/c4b_marker" -delete 2>/dev/null || true
find "${PS}/workspace/install" -type d -name '__pycache__' -empty -delete 2>/dev/null || true
echo "[C4b] 已清理 /root/.ros 与 launch 期间产生的 .pyc"

# -----------------------------------------------------------------------------
hdr "断言汇总"
if [ "${fail}" -eq 0 ]; then
  echo "[ASSERT] 全部通过（C1a / C1b / C1c / C2a / C2b / C3 / C4a / C4b）"
  exit 0
fi
echo "[ASSERT][FAIL] 存在失败断言，见上方各段"
exit 1
