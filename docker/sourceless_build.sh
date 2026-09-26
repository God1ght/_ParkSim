#!/usr/bin/env bash
# =============================================================================
# sourceless_build.sh —— 档 2：把自研运行期闭包编译为 Cython 原生扩展（.so），
# 并使交付镜像里不再存在可读的 Python 源码。
#
# 执行位置：**构建阶段**（有 gcc / python3-dev / cython），在 colcon build 之后。
# 一次性完成 6 步，任一步失败即中止构建：
#   S1  入口改写：把「会被直接 exec 的业务实现」整体搬进包，原地只留 6 行 stub
#   S2  重算静态可达闭包（AST 传递闭包，非「跑一遍」），并区分「待编译 / 保持源码」
#   S3  Cython 就地编译（-g0 减小体积）
#   S4  清中间产物（.c / 重复 build 目录 / __pycache__）并 strip 符号
#   S5  删除已编译模块的 .py，并裁掉闭包之外的包内源码（最小集）
#   S6  物化安装树：.so 落位 + 清掉由「符号链接农场」产生的悬空链接与真拷贝副本
#
# ── 为什么必须做 S6（本任务最反直觉的一点）──────────────────────────────────
#   ament_python_install_package 装出来的 install 树是**符号链接农场**：
#   实测 549 个软链、其中 152 个指向本脚本要删的源码。删了源码不物化安装树，
#   载入器会找不到模块；只删源码不删悬空链接，install 树看起来是坏的。
#   另外 install/.../parksim/webviz/ 是**真文件拷贝**（不是软链），
#   里面另有一份完整的 server.py —— 只处理源码树会漏掉这第二副本。
#
# ── 哪些东西必须保持源码（不可编译）────────────────────────────────────────
#   · *.launch.py          —— `ros2 launch` 要 exec 它们
#   · parksim/webviz/server.py —— run_app.sh 直接 exec 它（6 行 stub，零逻辑）
#   · msg/ srv/            —— rosidl 生成物，不属于自研
#
# ── 用法 ───────────────────────────────────────────────────────────────────
#   bash sourceless_build.sh                 # 使用 $PARKSIM_ROOT
#   bash sourceless_build.sh /path/to/_ParkSim
# =============================================================================
set -euo pipefail

ROOT="${1:-${PARKSIM_ROOT:?需要 PARKSIM_ROOT 或传入仓库根路径}}"
PY="$ROOT/python"
PKG="$PY/parksim"
INST="$ROOT/workspace/install/parksim/lib/python3.8/site-packages/parksim"
NODE_SRC="$ROOT/workspace/src/parksim/src"

log() { printf '\033[1;36m[sourceless]\033[0m %s\n' "$*"; }
die() { printf '\033[1;31m[sourceless][FATAL]\033[0m %s\n' "$*" >&2; exit 1; }

[ -d "$PKG" ]      || die "找不到包目录 $PKG"
[ -d "$INST" ]     || die "找不到安装树 $INST（colcon build 是否已执行？）"
[ -d "$NODE_SRC" ] || die "找不到节点源码目录 $NODE_SRC"
command -v cythonize >/dev/null 2>&1 || die "缺少 cythonize（构建阶段应已 pip install cython）"

# 入口文件必须保持可执行源码：run_app.sh 直接 exec 它。
KEEP_SRC='parksim.webviz.server'

# -----------------------------------------------------------------------------
# S1 入口改写
# -----------------------------------------------------------------------------
log "S1 入口改写：实现搬进包，原地留 stub"
mkdir -p "$PKG/nodes"
: > "$PKG/nodes/__init__.py"
for n in simulator vehicle visualizer; do
  f="$NODE_SRC/${n}_node.py"
  [ -f "$f" ] || die "缺少节点入口 $f"
  # 幂等：已经是 stub 就不再搬（重复执行本脚本时不会把 stub 当成实现搬走）
  if grep -q 'parksim\.nodes\.' "$f" 2>/dev/null; then
    log "  ${n}_node.py 已是 stub，跳过"
    continue
  fi
  cp -p "$f" "$PKG/nodes/${n}_node.py"
  cat > "$f" <<STUB
#!/usr/bin/env python3
"""ParkSim ${n} 节点入口；实现已编译，见 parksim.nodes.${n}_node。"""
from parksim.nodes.${n}_node import main

if __name__ == '__main__':
    main()
STUB
  chmod 755 "$f"
done
if ! grep -q 'parksim\.webviz\.server_impl' "$PKG/webviz/server.py" 2>/dev/null; then
  cp -p "$PKG/webviz/server.py" "$PKG/webviz/server_impl.py"
  cat > "$PKG/webviz/server.py" <<'STUB'
#!/usr/bin/env python3
"""ParkSim webviz 桥入口；实现已编译，见 parksim.webviz.server_impl。"""
from parksim.webviz.server_impl import main

if __name__ == '__main__':
    main()
STUB
  chmod 755 "$PKG/webviz/server.py"
fi
for f in "$NODE_SRC/vehicle_node.py" "$NODE_SRC/simulator_node.py" "$PKG/webviz/server.py"; do
  printf '  %3s 行  %s\n' "$(wc -l < "$f")" "${f#$ROOT/}"
done

# -----------------------------------------------------------------------------
# S2 重算静态可达闭包
# -----------------------------------------------------------------------------
log "S2 重算静态可达闭包（AST；覆盖配置可切换的全部路径）"
cat > /tmp/_parksim_closure.py <<'PYEOF'
import ast, os, sys
BASE=sys.argv[1]; PYROOT=os.path.join(BASE,'python'); PKG='parksim'
def mod_of(p):
    parts=os.path.relpath(p,PYROOT).split(os.sep)
    if parts[-1]=='__init__.py': parts=parts[:-1]
    else: parts[-1]=parts[-1][:-3]
    return '.'.join(parts)
def resolve(mod):
    parts=mod.split('.')
    if not parts or parts[0]!=PKG: return None
    p=os.path.join(PYROOT,*parts)
    if os.path.isfile(p+'.py'): return p+'.py'
    init=os.path.join(p,'__init__.py')
    if os.path.isdir(p) and os.path.isfile(init): return init
    return None
def deps(path):
    try: tree=ast.parse(open(path,encoding='utf-8',errors='replace').read(),filename=path)
    except SyntaxError: return set()
    out=set()
    for n in ast.walk(tree):
        if isinstance(n,ast.Import):
            for a in n.names: out.add(a.name)
        elif isinstance(n,ast.ImportFrom):
            if n.level:
                pp=mod_of(path).split('.')[:-1]; k=len(pp)-(n.level-1); pp=pp[:max(k,1)]
                b='.'.join(pp+([n.module] if n.module else []))
                out.add(b)
                for a in n.names: out.add(b+'.'+a.name)
            elif n.module:
                out.add(n.module)
                for a in n.names: out.add(n.module+'.'+a.name)
    return out
SEEDS=[os.path.join(PYROOT,'parksim/webviz/server.py')]+[
 os.path.join(BASE,'workspace/src/parksim/src/%s_node.py'%n) for n in ('simulator','vehicle','visualizer')]+[
 os.path.join(BASE,'workspace/src/parksim/launch/%s.launch.py'%n) for n in ('parksim','vehicle')]
seen,q=set(),list(SEEDS)
while q:
    c=q.pop()
    if c in seen or not os.path.isfile(c): continue
    seen.add(c)
    for d in deps(c):
        p=resolve(d)
        if p and p not in seen: q.append(p)
for p in sorted(seen):
    if p.startswith(PYROOT+os.sep) and os.path.basename(p)!='__init__.py':
        print(mod_of(p))
PYEOF
python3 /tmp/_parksim_closure.py "$ROOT" > /tmp/_closure_all.txt
printf '%s\n' "$KEEP_SRC" > /tmp/_keep.txt
grep -vxF -f /tmp/_keep.txt /tmp/_closure_all.txt > /tmp/_compile.txt
ALL=$(wc -l < /tmp/_closure_all.txt); CMP=$(wc -l < /tmp/_compile.txt)
log "  闭包 $ALL 个（保持源码 1 个：$KEEP_SRC，待编译 $CMP 个）"
[ "$CMP" -ge 10 ] || die "待编译模块数异常（$CMP），闭包计算可能失败"

# -----------------------------------------------------------------------------
# S3 编译
# -----------------------------------------------------------------------------
log "S3 Cython 编译（-g0，annotation_typing=False 保语义）"
# ── 为什么必须显式关掉 annotation_typing（这是一个实测出来的坑）──────────────
#   Cython 3 默认 annotation_typing=True：把源码里的 Python 注解当成**运行时类型
#   检查**。本仓库的 stanley_controller.set_ref_pose 写着
#       def set_ref_pose(self, x_ref: List[float], ...)
#   而调用方 rule_based_stanley_vehicle.py 明确 "预转 numpy"，于是编译产物在
#   `ndarray` 入参处抛
#       TypeError: Argument 'x_ref' has incorrect type (expected list, got numpy.ndarray)
#   结果：出库/入库车辆逐个崩死、仿真进程随之退出（实测 max_vehicles 3→2）。
#   纯 Python 下注解是惰性的，所以这类代码在 CPython 里完全正常 —— 关掉该指令
#   是让编译产物与 CPython 语义一致的必要条件。
#   --force 也是必须的：cythonize 在 .c 比 .py 新时会**复用旧 .c**，
#   增量构建下换了指令却不重新生成，等于指令没生效（也踩过）。
ok=0; fail=0; t0=$SECONDS
while read -r m; do
  [ -z "$m" ] && continue
  f="$PY/$(echo "$m" | tr '.' '/').py"
  [ -f "$f" ] || die "编译清单里的模块找不到源文件：$m"
  if CFLAGS="-g0" cythonize -i -3 -q --force -X annotation_typing=False "$f" >>/tmp/_cython.log 2>&1; then
    ok=$((ok+1)); echo "  OK   $m"
  else
    fail=$((fail+1)); echo "  FAIL $m"; tail -25 /tmp/_cython.log >&2
  fi
done < /tmp/_compile.txt
log "  编译 ok=$ok fail=$fail 用时 $((SECONDS-t0))s"
[ "$fail" -eq 0 ] || die "有 $fail 个模块编译失败（见上方日志）—— 不得带着断臂的交付物继续"

# -----------------------------------------------------------------------------
# S4 清理中间产物
# -----------------------------------------------------------------------------
log "S4 清理 .c / 重复 build 目录 / __pycache__，并 strip 符号"
# Cython 默认会把源码以注释形式写进生成的 .c；留一份 .c 就等于留一份源码。
find "$PKG" -name '*.c' -delete 2>/dev/null || true
find "$PKG" -type d -name 'build' -prune -exec rm -rf {} + 2>/dev/null || true
find "$PKG" -type d -name '*.egg-info' -prune -exec rm -rf {} + 2>/dev/null || true
find "$PKG" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true
while read -r m; do
  [ -z "$m" ] && continue
  for so in "$PY/$(echo "$m" | tr '.' '/')".*.so; do
    [ -f "$so" ] && strip --strip-unneeded "$so" 2>/dev/null || true
  done
done < /tmp/_compile.txt
log "  剩余 .c=$(find "$PKG" -name '*.c' | wc -l) 个 .so=$(find "$PKG" -name '*.so' | wc -l) 个 体积=$(du -sh "$PKG" | cut -f1)"

# -----------------------------------------------------------------------------
# S5 删源码 + 裁最小集
# -----------------------------------------------------------------------------
log "S5 删除已编译模块 .py，并裁掉闭包之外的包内源码"
nd=0
while read -r m; do
  [ -z "$m" ] && continue
  f="$PY/$(echo "$m" | tr '.' '/').py"
  [ -f "$f" ] && { rm -f "$f"; nd=$((nd+1)); }
done < /tmp/_compile.txt
log "  已删 $nd 个已编译模块的 .py"

# 最小集：以「目录级」而不是「文件级」为单位裁剪。
#   · 不含任何闭包模块的包目录 —— 整棵删除。否则会留下"只剩 __init__.py 与
#     说明文档"的空壳（trajectory_predict/、intent_predict/、simulator/、
#     spot_detector/、map/ 都是这一类），既占位又夹带可读文本。
#   · 保留目录内，非闭包的 .py 逐个删除（visualizer/offline_visualizer.py、
#     path_planner/hybrid_astar/{car,hybrid_a_star,dynamic_programming_heuristic}.py、
#     path_planner/hobca_planner.py、utils/generate_profiles.py 等）。
#   · 整棵保留的例外只有两个：
#       priorFiles/ —— 地图与 pickle 数据资产（**实测无任何 .py**，纯数据）
#       webviz/     —— 前端静态资源 + 入口 stub + 已编译的 server_impl .so
python3 - "$PKG" /tmp/_closure_all.txt <<'PYEOF'
import os, sys, shutil
PKG = os.path.abspath(sys.argv[1]); LIST = sys.argv[2]
closure = [l.strip() for l in open(LIST) if l.strip()]

keep_files, keep_dirs = set(), set()
def add_ancestors(d):
    d = os.path.abspath(d)
    while True:
        keep_dirs.add(d)
        if d == PKG or os.path.dirname(d) == d:
            break
        d = os.path.dirname(d)
for m in closure:
    parts = m.split('.')[1:]                       # 去掉包名 parksim
    if len(parts) > 1:
        add_ancestors(os.path.join(PKG, *parts[:-1]))
    else:
        keep_dirs.add(PKG)
    rel = os.path.join(*parts) if parts else ''
    keep_files.add(os.path.join(PKG, rel + '.py'))
    keep_files.add(os.path.join(PKG, rel, '__init__.py'))
keep_files.add(os.path.join(PKG, 'webviz', 'server.py'))

EXC_DIRS = [os.path.join(PKG, 'priorFiles'), os.path.join(PKG, 'webviz')]
def in_exc(p):
    p = os.path.abspath(p)
    return any(p == e or p.startswith(e + os.sep) for e in EXC_DIRS)

# 1) 目录级（自底向上）：删除不含任何闭包模块的目录，例外目录除外
removed_dirs = []
for r, _d, _f in os.walk(PKG, topdown=False):
    if os.path.abspath(r) == PKG or in_exc(r):
        continue
    if os.path.abspath(r) in keep_dirs:
        continue
    n = sum(len(f) for _rr, _dd, f in os.walk(r))
    rel = os.path.relpath(r, PKG)
    shutil.rmtree(r)
    removed_dirs.append((rel, n))

# 2) 文件级：保留目录内，非闭包的 .py 逐个删除
removed_files = []
for r, _d, files in os.walk(PKG):
    if in_exc(r):
        continue
    for x in files:
        if not x.endswith('.py') or x == '__init__.py':
            continue
        p = os.path.join(r, x)
        if p in keep_files:
            continue
        os.remove(p)
        removed_files.append(os.path.relpath(p, PKG))

print('  [S5] 整棵删除的无闭包目录 %d 个：' % len(removed_dirs))
for rel, n in sorted(removed_dirs):
    print('        %-42s（%d 个文件）' % (rel, n))
print('  [S5] 保留目录内单独删除的非闭包 .py = %d 个' % len(removed_files))
for rel in sorted(removed_files):
    print('        %s' % rel)

# 3) webviz/ 里零引用的历史副本：浏览器拿到的是 static/ 下那一份，外边这些是旧版
#    （安装树那份由 S6 处理）。只删显式列举的文件，不做通配。
for x in ('app.js', 'index.html', 'run_web_sim.sh', 'README.md'):
    p = os.path.join(PKG, 'webviz', x)
    if os.path.exists(p):
        os.remove(p)
        print('  [S5] 删除 webviz 历史副本 %s' % x)

# 4) 源码型**非 .py** 文件：notebook / Cython 源 / 备份。
#    为什么必须单独处理：notebook 里就是 Python 源码（JSON 形式），而它所在的目录
#    可能属于闭包（route_planner/test_planner.ipynb 就是这样活下来的）。
#    实测交付子树里有 8 个 .ipynb，其中 7 个在被整棵删除的目录里，1 个在保留目录里。
#    注意 **不能删 .so**（那是本脚本 S3 的产物）。
SRC_LIKE = ('.ipynb', '.pyx', '.pxd', '.pxi', '.pyi', '.c', '.cpp', '.h', '.hpp')
removed_src = []
for r, dirs, files in os.walk(PKG):
    if in_exc(r):
        dirs[:] = [d for d in dirs if d != '__ipynb_checkpoints']
        continue
    if '__ipynb_checkpoints' in dirs:
        shutil.rmtree(os.path.join(r, '__ipynb_checkpoints'), ignore_errors=True)
    for x in files:
        if x.endswith(SRC_LIKE) or '.bak' in x or x.endswith(('~', '.orig', '.rej', '.swp')):
            p = os.path.join(r, x)
            os.remove(p)
            removed_src.append(os.path.relpath(p, PKG))
print('  [S5] 删除源码型非 .py 文件 %d 个：' % len(removed_src))
for rel in sorted(removed_src):
    print('        %s' % rel)
PYEOF

REST=$(find "$PKG" -name '*.py' ! -name '__init__.py' | wc -l)
log "  包内仍为可读 .py 的非 __init__ 文件 = $REST（应为 1，即 webviz/server.py stub）"
[ "$REST" -le 1 ] || { find "$PKG" -name '*.py' ! -name '__init__.py'; die "包内仍有 $REST 个可读源码，最小集裁剪不完整"; }

# -----------------------------------------------------------------------------
# S6 物化安装树
# -----------------------------------------------------------------------------
log "S6 物化安装树"
# 先 rm 再 cp：安装树里目标可能是「指向源码树的软链」或「真拷贝」，两种都必须
# 变成真文件。直接 cp -f 会顺着软链写到源码树里去，留下一个指回去的软链。
( cd "$PKG" && find . -name '*.so' | while read -r s; do
    d=$(dirname "$s"); mkdir -p "$INST/$d"
    rm -f "$INST/$s"; cp -f "$s" "$INST/$s"; done )
# 已编译模块在安装树里的 .py（可能是悬空软链，也可能是 webviz/ 那种真拷贝）一律删除
while read -r m; do
  [ -z "$m" ] && continue
  rm -f "$INST/$(echo "$m" | sed 's/^parksim\.//; s/\./\//g').py"
done < /tmp/_compile.txt
# 安装树里 webviz/ 的 server.py 是真拷贝（内含一份完整实现）→ 换成 stub。
# 同样先 rm 再 cp（目标若是软链，cp -f 会写到源码树）。
rm -f "$INST/webviz/server.py"
cp -f "$PKG/webviz/server.py" "$INST/webviz/server.py"
# ── 安装树里"源码树中不存在"的残留包：整棵删除 ──────────────────────────────
#   实测存在一个历史遗留的 parksim/rl/（12 个模块、960 行可读 Python，文件日期
#   早于当前源码树，源码树里已无 rl/）。只清源码树**碰不到它** —— install 树里
#   那份是**真文件拷贝**而非软链。这类"只存在于 install 树的旧包"只能在这里清。
#   msg/ 与 srv/ 是 rosidl 生成物（源码树里没有、必须在安装树里保留），明确豁免。
for d in "$INST"/*/; do
  [ -d "$d" ] || continue
  b="$(basename "$d")"
  case "$b" in msg|srv|__pycache__|static) continue ;; esac
  if [ ! -e "$PKG/$b" ]; then
    log "  删除安装树残留包 $b（源码树里不存在，$(du -sh "$d" | cut -f1)，$(find "$d" -type f | wc -l) 个文件）"
    rm -rf "$d"
  fi
done
# 其余"真文件 .py"（非软链）除 __init__.py 与入口 stub 外一律清掉（兜底）。
# 只删真文件：msg/ srv/ 下那份 rosidl 包装是软链（运行期必需，见 S7 的登记例外）。
find "$INST" -name '*.py' ! -type l ! -name '__init__.py' ! -path "$INST/webviz/server.py" \
     -delete 2>/dev/null || true
# 零引用旧前端与启动脚本（真拷贝，不会被软链清理覆盖）
rm -f "$INST/webviz/app.js" "$INST/webviz/index.html" "$INST/webviz/run_web_sim.sh" "$INST/webviz/README.md"
find "$INST" -type d -name '__pycache__' -prune -exec rm -rf {} + 2>/dev/null || true
DANGLE_BEFORE=$(find "$INST" -xtype l | wc -l)
find "$INST" -xtype l -delete
DANGLE_AFTER=$(find "$INST" -xtype l | wc -l)
log "  悬空软链 $DANGLE_BEFORE -> $DANGLE_AFTER（应归零）"
[ "$DANGLE_AFTER" -eq 0 ] || die "安装树仍有 $DANGLE_AFTER 个悬空软链"

# -----------------------------------------------------------------------------
# S7 构建期静态断言
# -----------------------------------------------------------------------------
log "S7 构建期静态断言"
python3 - "$ROOT" /tmp/_compile.txt <<'PYEOF'
import os, sys
ROOT, LIST = sys.argv[1], sys.argv[2]
INST=os.path.join(ROOT,'workspace/install/parksim/lib/python3.8/site-packages/parksim')
PKG=os.path.join(ROOT,'python/parksim')
mods=[l.strip() for l in open(LIST) if l.strip()]
bad=[]
for m in mods:
    rel=m[len('parksim.'):].replace('.','/')
    d=os.path.dirname(os.path.join(INST,rel)); base=os.path.basename(rel)
    if not os.path.isdir(d): bad.append(('no-dir',m)); continue
    names=os.listdir(d)
    if not any(n.startswith(base+'.') and n.endswith('.so') for n in names): bad.append(('no-so',m))
    if os.path.exists(os.path.join(INST,rel+'.py')): bad.append(('py-left',m))
if bad:
    print('  [S7] 失败明细：', bad); sys.exit(1)
print('  [S7] %d 个模块在安装树里都有 .so 且无 .py 残留' % len(mods))

# 安装树里不得存在可读源码。注意这不是"源码树的推论"：install 树是符号链接农场，
# 但 webviz/ 与 rl/ 这类目录是**真文件拷贝** —— 只清源码树会在这里漏掉第二副本。
#   已登记例外（不是遗漏，是必须保留）：
#     msg/ srv/ —— rosidl 自动生成的接口包装（15 个 msg + 1 个 srv），运行期必需，
#                  内容为机器生成的样板，不含手写逻辑；它们是指向 workspace/build
#                  的软链。这里只登记**前缀**，并单独报出数量，防止被人顺手放宽。
ALLOWED_PREFIX = ('msg/', 'srv/')
EXC = {'__init__.py', 'webviz/server.py'}
leak, allowed = [], 0
for r, _d, f in os.walk(INST):
    for x in f:
        if not x.endswith('.py'): continue
        p = os.path.join(r, x); rel = os.path.relpath(p, INST)
        if rel in EXC or x == '__init__.py':
            continue
        if rel.startswith(ALLOWED_PREFIX):
            allowed += 1; continue
        if os.path.islink(p) and not os.path.exists(p): continue    # 悬空链，S6 已清
        leak.append((rel, ('symlink->%s' % os.readlink(p)) if os.path.islink(p) else 'real'))
if leak:
    print('  [S7] 安装树里仍有可读源码 %d 个：' % len(leak))
    for x in leak[:15]: print('        ', x)
    sys.exit(1)
print('  [S7] 安装树无可读源码（除 __init__.py、入口 stub 与已登记的 rosidl msg/srv %d 个外）' % allowed)

# 备份/临时文件 + 源码型非 .py 文件守卫。
#   `-name '*.py'` 匹配不到 `*.py.bak_*` 与 `*.ipynb`，必须单独查。
#   （本仓库活树里有 66 个 .bak 文件；git archive 里为 0，交付子树里有 8 个 .ipynb。
#     这条是防回归的兜底，S5 第 4 步负责实际删除。）
SRC_LIKE = ('.ipynb', '.pyx', '.pxd', '.pxi', '.pyi', '.c', '.cpp', '.h', '.hpp')
junk = []
for r, _d, f in os.walk(PKG):
    for x in f:
        if x.endswith(SRC_LIKE) or '.bak' in x \
           or x.endswith(('~', '.orig', '.rej', '.swp', '.pyc')) or x.startswith('.#'):
            junk.append(os.path.relpath(os.path.join(r, x), PKG))
if junk:
    print('  [S7] 包内仍有源码型非 .py / 备份文件 %d 个：%s' % (len(junk), junk[:10])); sys.exit(1)
print('  [S7] 包内源码型非 .py / 备份文件 = 0')

# 顺带报出"除 priorFiles/ 与 webviz/static/ 之外，还有哪些文件随镜像发出"。
# 现在这条已可当闸门：实测交付子树里该类文件只有 8 个 .ipynb + 2 个 .eps + 2 个
# .dict + trajectory_predict/README.md + webviz 的 4 个历史副本，全部应被 S5 清掉；
# 清完后除 webviz/static/（浏览器本来就要下载的前端）之外应**一个不剩**。
EXC = [os.path.join(PKG, 'priorFiles'), os.path.join(PKG, 'webviz', 'static')]
def in_exc(p):
    p = os.path.abspath(p)
    return any(p == e or p.startswith(e + os.sep) for e in EXC)
leftover = []
for r, _d, f in os.walk(PKG):
    if in_exc(r): continue
    for x in f:
        p = os.path.join(r, x); rel = os.path.relpath(p, PKG)
        if x == '__init__.py' or x.endswith('.so') or rel == 'webviz/server.py': continue
        leftover.append(rel)
if leftover:
    print('  [S7] FAIL priorFiles/ 与 webviz/static/ 之外仍有 %d 个文件随镜像发出：' % len(leftover))
    for rel in sorted(leftover): print('         %s' % rel)
    sys.exit(1)
print('  [S7] priorFiles/ 与 webviz/static/ 之外随镜像发出的文件 = 0')
c=sum(1 for r,d,f in os.walk(PKG) for x in f if x.endswith('.c'))
assert c==0, '包内残留 %d 个 .c（等于泄漏源码）' % c
print('  [S7] 包内 .c 残留 = 0')
src=os.path.join(PKG,'webviz/server.py')
n=sum(1 for _ in open(src,encoding='utf-8'))
assert n<=10, '入口 stub 有 %d 行，超限' % n
print('  [S7] 入口 stub %d 行' % n)
PYEOF

# 记录清单，供运行阶段独立复核（只含模块名与计数，不含源码）
mkdir -p /opt/parksim
cp /tmp/_compile.txt /opt/parksim/CORE_MODULES.txt
{
  echo "sourceless_core_modules : $(wc -l < /tmp/_compile.txt)"
  echo "kept_source_entries     : ${KEEP_SRC}"
  echo "cython_ok               : ${ok}"
  echo "cython_fail             : ${fail}"
  echo "dangling_symlinks_pruned: ${DANGLE_BEFORE}"
  echo "pkg_size_after          : $(du -sh "$PKG" | cut -f1)"
} > /opt/parksim/SOURCELESS.txt
cat /opt/parksim/SOURCELESS.txt
log "完成"
