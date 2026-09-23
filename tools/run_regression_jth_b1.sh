#!/usr/bin/env bash
# ParkSim JTH — jth_b1 60 车回归（多出入口 / 有向图）
#
# 门禁：完成数 ≥ 59。
# 注意：旧基线 59/60 是在「假入口」下取得的（spawn/离场目标都落在顶点 291，
# P1/P2/P3/P4 从未被使用）。切换真出入口后完成数可能短期波动，
# 因此本脚本以「新基线 ≥59」为准，并额外统计出入口实际分布。
#
# 用法（在 Mac 上）：
#   ssh step@172.16.1.167 'bash /media/step/data/Yccc7/ParkSim-JTH/_ParkSim/tools/run_regression_jth_b1.sh'
# 可选参数：
#   $1 输出日志路径
#   $2 完成数门槛（默认 59）
# 注意：set -u 必须在 source 之后 —— conda/ROS 环境脚本内含未绑定变量，
# 非交互 bash 在 set -u 下 source 它们会直接终止（实测踩过）。
source /media/step/data/Yccc7/ParkSim-JTH/env_ros.sh >/dev/null 2>&1

set -u

ROOT=/media/step/data/Yccc7/ParkSim-JTH/_ParkSim
LOG="${1:-$ROOT/verify_jth_$(date +%Y%m%d_%H%M%S).log}"
THRESH="${2:-59}"

cd "$ROOT" || exit 2

timeout 1800 ros2 launch parksim parksim.launch.py map:=jth_b1 gui:=false > "$LOG" 2>&1
rc=$?

exited=$(grep -c 'exit vehicle .* ended' "$LOG")
parked=$(grep -c 'parking vehicle .* ended' "$LOG")
added=$(grep -c 'is added with spot_index' "$LOG")
total=$((exited + parked))

# 出入口实际分布（vehicle_node 每车会打 entry_portal / exit_portal）
e_p1=$(grep -c 'entry_portal=P1' "$LOG")
e_p4=$(grep -c 'entry_portal=P4' "$LOG")
x_p1=$(grep -c 'exit_portal=P1' "$LOG")
x_p2=$(grep -c 'exit_portal=P2' "$LOG")
x_p3=$(grep -c 'exit_portal=P3' "$LOG")

# 死锁修复特异性观测：离场车不得出现越界爬行
out_of_bounds=$(grep -cE 'Vehicle [0-9]+ .*x=[0-9]{3,}' "$LOG")

echo "log=$LOG"
echo "launch_exit=$rc"
echo "vehicles_added=$added"
echo "exited=$exited parked=$parked completed=$total (threshold=$THRESH)"
echo "entry_portal: P1=$e_p1 P4=$e_p4"
echo "exit_portal:  P1=$x_p1 P2=$x_p2 P3=$x_p3"
echo "suspect_out_of_bounds_lines=$out_of_bounds"
echo "VERDICT=$([ "$total" -ge "$THRESH" ] && echo PASS || echo FAIL)"
