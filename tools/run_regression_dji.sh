#!/usr/bin/env bash
# ParkSim JTH — DJI_0001 60 车回归（legacy 行为门禁）
#
# 用途：验证「多出入口 / 有向图」改造未破坏 DJI（无 portals → 走 legacy 分支）。
# 门禁：完成数 = 60（exit vehicle N ended + parking vehicle N ended），且 0 失败。
#
# 用法（在 Mac 上）：
#   ssh step@172.16.1.167 'bash /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/tools/run_regression_dji.sh'
# 或先 scp 上传本文件后执行。可选参数：
#   $1 输出日志路径（默认 .../verify_dji_<时间戳>.log）
# 注意：set -u 必须在 source 之后 —— conda/ROS 环境脚本内含未绑定变量，
# 非交互 bash 在 set -u 下 source 它们会直接终止（实测踩过）。
source /media/step/data/Yccc7/ParkSim-JTH/env_ros.sh >/dev/null 2>&1

set -u

ROOT=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim
LOG="${1:-$ROOT/verify_dji_$(date +%Y%m%d_%H%M%S).log}"

cd "$ROOT" || exit 2

# 硬超时，避免挂死；一轮 60 车约 12 分钟
timeout 1500 ros2 launch parksim parksim.launch.py map:=DJI_0001 gui:=false > "$LOG" 2>&1
rc=$?

exited=$(grep -c 'exit vehicle .* ended' "$LOG")
parked=$(grep -c 'parking vehicle .* ended' "$LOG")
added=$(grep -c 'is added with spot_index' "$LOG")
total=$((exited + parked))

echo "log=$LOG"
echo "launch_exit=$rc"
echo "vehicles_added=$added"
echo "exited=$exited parked=$parked completed=$total"
echo "VERDICT=$([ "$total" -ge 60 ] && echo PASS || echo FAIL)"
