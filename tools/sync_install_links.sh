#!/usr/bin/env bash
# ParkSim install 树符号链接同步工具（幂等）
#
# 背景：运行时 import parksim / ros2 launch 读取的 install 树是"真实目录 + 文件级符号链接"。
#       仓库新增文件后，必须在 install/share 树补建同名符号链接才生效。
#
# 用法（在 _ParkSim 根目录或任意位置）：
#   bash tools/sync_install_links.sh                 # 同步默认列表（新增文件都在这里）
#   bash tools/sync_install_links.sh <文件相对路径>... # 同步指定文件（可传多个）
set -euo pipefail

ROOT="${PARKSIM_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
INST="$ROOT/workspace/install/parksim"

sync_one() {
  local rel="$1"
  local src="$ROOT/$rel"
  local dst=""
  case "$rel" in
    python/parksim/*)
      dst="$INST/lib/python3.8/site-packages/parksim/${rel#python/parksim/}" ;;
    workspace/src/parksim/config/*)
      dst="$INST/share/parksim/config/${rel#workspace/src/parksim/config/}" ;;
    workspace/src/parksim/launch/*)
      dst="$INST/share/parksim/launch/${rel#workspace/src/parksim/launch/}" ;;
    workspace/src/parksim/src/*)
      dst="$INST/lib/parksim/${rel#workspace/src/parksim/src/}" ;;
    *)
      echo "[sync] 跳过（未知路径规则）: $rel"; return 0 ;;
  esac
  if [ ! -e "$src" ]; then echo "[sync] 源不存在（跳过）: $src"; return 0; fi
  mkdir -p "$(dirname "$dst")"
  if [ -L "$dst" ] && [ "$(readlink "$dst")" = "$src" ]; then
    echo "[sync] 已存在 : ${dst#"$ROOT"/}"; return 0
  fi
  if [ -e "$dst" ] && [ ! -L "$dst" ]; then
    echo "[sync] 警告：目标已存在且非符号链接，跳过: $dst"; return 1
  fi
  ln -sfn "$src" "$dst"
  echo "[sync] 已链接 : ${dst#"$ROOT"/} -> $src"
}

defaults=(
  "python/parksim/allocation.py"
  "python/parksim/route_planner/ref_path_generator.py"
  "python/parksim/path_planner/maneuver_providers.py"
  "python/parksim/scenario.py"
  "workspace/src/parksim/config/scenario.yaml"
  "workspace/src/parksim/config/scenarios"
  "workspace/src/parksim/config/maps"
)

if [ $# -gt 0 ]; then items=("$@"); else items=("${defaults[@]}"); fi
for it in "${items[@]}"; do sync_one "$it"; done
echo "[sync] 完成"
