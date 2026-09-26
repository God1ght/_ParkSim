#!/usr/bin/env bash
# =============================================================================
# assert_sourceless.sh —— 档 2 的**运行阶段**护栏：证明"交付物里没有可读源码、
# 且每个编译产物在本阶段的库里都真的能加载"。
#
# 为什么必须放在运行阶段而不是构建阶段：
#   构建阶段有完整工具链与全量 pip 依赖，运行阶段只留实测出的运行期依赖。
#   某个模块可能在构建阶段导得进去、在运行阶段因为缺一个库而炸 —— 这类缺口
#   在构建阶段是**隐形**的。所以「能不能导入」这条断言必须在最终阶段成立才算数。
#
# 断言清单（C5 系列）：
#   C5a  CORE_MODULES.txt 里每个模块都从**安装树**导入成功，且 __file__ 以 .so 结尾
#   C5b  parksim.msg / parksim.srv（rosidl 生成物）仍可导入
#   C5c  保证例外模块（依赖未安装的 dearpygui）显式列出，且失败原因必须正是它
#   C5d  入口 stub：parksim.webviz.server 解析到 <=10 行的 .py（源码树与安装树各一份）
#   C5e  三个节点可执行仍存在于 install/parksim/lib/parksim/ 且指向 stub
#   C5f  包内不存在任何非 __init__ 的可读 .py（stub 除外）；不存在 .c
#   C5g  安装树无悬空软链
#   C5h  安装树里不存在可读源码（rosidl msg/srv 为登记例外）
#
# 用法：bash assert_sourceless.sh
# =============================================================================
set -uo pipefail

ROOT="${PARKSIM_ROOT:-/media/step/data/Yccc7/ParkSim-JTH/_ParkSim}"
INST="$ROOT/workspace/install/parksim/lib/python3.8/site-packages/parksim"
LIBD="$ROOT/workspace/install/parksim/lib/parksim"
PKG="$ROOT/python/parksim"
LIST="${CORE_MODULES_LIST:-/opt/parksim/CORE_MODULES.txt}"

pass=0; fail=0
ok()   { printf '  \033[1;32m[PASS]\033[0m %s\n' "$*"; pass=$((pass+1)); }
no()   { printf '  \033[1;31m[FAIL]\033[0m %s\n' "$*" >&2; fail=$((fail+1)); }

echo "== C5 源码可读性与编译产物完整性（档 2） =="
[ -f "$LIST" ] || { no "C5a 缺少 $LIST（镜像不是经 sourceless_build.sh 构建的？）"; echo "C5 结果: pass=$pass fail=$fail"; exit 1; }
echo "  清单：$LIST（$(wc -l < "$LIST") 个模块）"

# --- 进入应用自身的运行环境 -------------------------------------------------
set +u
# shellcheck disable=SC1090
source "${ROS_FOXY_ROOT}/setup.bash" >/dev/null 2>&1
# shellcheck disable=SC1090
source "$ROOT/workspace/install/setup.bash" >/dev/null 2>&1
set -u
export PYTHONPATH="${DLP_DATASET_ROOT:-}:${PYTHONPATH:-}"

# --- C5a / C5b / C5c / C5d --------------------------------------------------
cd /   # 关键：cwd 不能是源码树，否则会把源码树当成解析目标，断言就失去意义
python3 - "$INST" "$LIST" <<'PY'
import importlib, os, sys
INST, LIST = sys.argv[1], sys.argv[2]
mods=[l.strip() for l in open(LIST) if l.strip()]
# 例外：这两个模块在模块级 import dearpygui（3D 可视化窗口用），而镜像从不安装它
# （v2 同样如此，属既有事实）。visualizer_node 仅在 gui:=true 时启动。
COND={'parksim.nodes.visualizer_node':'dearpygui',
      'parksim.visualizer.realtime_visualizer':'dearpygui'}

p=importlib.import_module('parksim')
if not str(p.__file__).startswith(INST):
    print('[C5a] FAIL parksim 未从安装树解析：%s' % p.__file__); sys.exit(1)
print('  [C5a] parksim 解析自安装树：%s' % p.__file__)

bad=[]; cond=[]; okc=0
for m in mods:
    try:
        mm=importlib.import_module(m)
        f=getattr(mm,'__file__','?')
        if not str(f).endswith('.so'):
            bad.append((m,'不是 .so：%s' % f))
        else:
            okc+=1
    except ModuleNotFoundError as e:
        want=COND.get(m)
        if want and want in str(e):
            cond.append(m)
        else:
            bad.append((m,'ModuleNotFoundError: %s' % e))
    except Exception as e:
        bad.append((m,'%s: %s' % (type(e).__name__, e)))
if bad:
    print('[C5a] FAIL %d 个模块未通过：' % len(bad))
    for b in bad: print('        ', b)
    sys.exit(1)
print('  [C5a] %d/%d 个模块从安装树导入且解析为 .so；例外 %d 个（%s）'
      % (okc, len(mods), len(cond), ','.join(cond) or '无'))
if set(cond) - set(COND):
    print('[C5c] FAIL 出现未登记的导入失败：%s' % (set(cond)-set(COND))); sys.exit(1)
print('  [C5c] 例外集合与登记一致（仅 dearpygui 依赖项）')

from parksim.msg import VehicleStateMsg, VehicleInfoMsg     # noqa: F401
from parksim.srv import OccupancySrv                        # noqa: F401
print('  [C5b] parksim.msg / parksim.srv（rosidl 生成物）可导入')

m=importlib.import_module('parksim.webviz.server')
f=getattr(m,'__file__','?')
n=sum(1 for _ in open(f,encoding='utf-8')) if os.path.isfile(f) else -1
if not str(f).endswith('.py') or n > 10:
    print('[C5d] FAIL 入口 stub 异常：%s（%s 行）' % (f, n)); sys.exit(1)
print('  [C5d] 源码树入口 stub 解析为 .py，%d 行' % n)
inst_srv=os.path.join(INST,'webviz','server.py')
if not os.path.isfile(inst_srv):
    print('[C5d] FAIL 安装树缺 webviz/server.py'); sys.exit(1)
n2=sum(1 for _ in open(inst_srv,encoding='utf-8'))
if n2 > 10:
    print('[C5d] FAIL 安装树 webviz/server.py 仍是完整实现（%d 行）' % n2); sys.exit(1)
print('  [C5d] 安装树 webviz/server.py 也是 stub，%d 行' % n2)

# C5h：安装树里不得存在可读源码。
# 为什么不能只信源码树那条断言：install 树大部分是**指向源码树的软链**，
# 但 webviz/、rl/ 一类的目录是**真文件拷贝**。只清源码树，第二副本会原样留下，
# 而那时源码树那边看起来完全干净 —— 这条断言就是补这个盲区的。
#   （实测就靠它抓到 install 树里一个历史遗留的 parksim/rl/：12 个模块、960 行可读
#    Python，源码树里早已没有 rl/，文件日期也早于当前源码树。）
# 已登记例外（必须保留，非遗漏）：msg/ 与 srv/ 下 rosidl 自动生成的接口包装
# （15 个 msg + 1 个 srv）。运行期必需、内容为机器生成样板、不含手写逻辑。
ALLOWED_PREFIX = ('msg/', 'srv/')
EXC_REL = {'webviz/server.py'}
leak, allowed = [], []
for _r, _d, _f in os.walk(INST):
    for x in _f:
        if not x.endswith('.py'): continue
        p = os.path.join(_r, x); rel = os.path.relpath(p, INST)
        if x == '__init__.py' or rel in EXC_REL: continue
        if rel.startswith(ALLOWED_PREFIX):
            allowed.append(rel); continue
        if os.path.islink(p) and not os.path.exists(p): continue   # 悬空链
        leak.append((rel, ('symlink->%s' % os.readlink(p)) if os.path.islink(p) else 'real'))
if leak:
    print('[C5h] FAIL 安装树里仍有可读源码 %d 个：' % len(leak))
    for x in leak[:15]: print('        ', x)
    sys.exit(1)
print('  [C5h] 安装树无可读源码（除 __init__.py、入口 stub 与已登记的 rosidl msg/srv %d 个外）'
      % len(allowed))
PY
[ $? -eq 0 ] && ok "C5a/C5b/C5c/C5d 通过" || no "C5a~C5d 有失败项"

# --- C5e 节点可执行 ----------------------------------------------------------
miss=0
for n in simulator vehicle visualizer; do
  f="$LIBD/${n}_node.py"
  if [ ! -e "$f" ]; then no "C5e 缺节点入口 $f"; miss=1; continue; fi
  lines=$(wc -l < "$f" 2>/dev/null || echo 999)
  if [ "$lines" -gt 10 ]; then no "C5e $f 有 $lines 行（应为 stub）"; miss=1; fi
done
[ "$miss" -eq 0 ] && ok "C5e 三个节点入口存在且均为 stub" || no "C5e 节点入口异常"

# --- C5f 可读源码与 .c 残留 --------------------------------------------------
resid=$(find "$PKG" -name '*.py' ! -name '__init__.py' | wc -l)
if [ "$resid" -eq 1 ]; then
  only=$(find "$PKG" -name '*.py' ! -name '__init__.py')
  case "$only" in
    */webviz/server.py) ok "C5f 包内仅剩 ${resid} 个可读 .py（入口 stub webviz/server.py）" ;;
    *) no "C5f 包内剩余的可读 .py 不是预期入口：$only" ;;
  esac
else
  no "C5f 包内仍有 $resid 个非 __init__ 的可读 .py"; find "$PKG" -name '*.py' ! -name '__init__.py' | head -10 >&2
fi
nc=$(find "$PKG" -name '*.c' | wc -l)
[ "$nc" -eq 0 ] && ok "C5f 包内无 .c 残留（.c 会把源码写成注释）" || no "C5f 包内残留 $nc 个 .c"

# --- C5g 悬空软链 ------------------------------------------------------------
d=$(find "$ROOT/workspace/install" -xtype l | wc -l)
[ "$d" -eq 0 ] && ok "C5g 安装树无悬空软链" || no "C5g 安装树有 $d 个悬空软链"

echo
if [ "$fail" -eq 0 ]; then
  printf '== C5 全部通过（pass=%d, fail=0）\n' "$pass"
  [ -f /opt/parksim/SOURCELESS.txt ] && sed 's/^/   /' /opt/parksim/SOURCELESS.txt
  exit 0
else
  printf '== C5 存在失败项（pass=%d, fail=%d）\n' "$pass" "$fail" >&2
  exit 1
fi
