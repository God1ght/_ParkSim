#!/usr/bin/env bash
# 统计 ParkSim 回归日志的完成情况
# 用法: bash tools/_count.sh <logfile> [threshold]
set -u
LOG="${1:?usage: _count.sh <logfile> [threshold]}"
THRESH="${2:-60}"

add=$(grep -c 'added with spot_index' "$LOG")
ex=$(grep -c 'exit vehicle' "$LOG" | cat)
ex=$(grep -cE 'exit vehicle [0-9]+ ended' "$LOG")
pk=$(grep -cE 'parking vehicle [0-9]+ ended' "$LOG")
tot=$((ex + pk))
noslot=$(grep -cE 'No free spot|No departable' "$LOG")
tb=$(grep -c 'Traceback' "$LOG")
ep1=$(grep -c 'entry_portal=P1' "$LOG")
ep4=$(grep -c 'entry_portal=P4' "$LOG")
xp1=$(grep -c 'exit_portal=P1' "$LOG")
xp2=$(grep -c 'exit_portal=P2' "$LOG")
xp3=$(grep -c 'exit_portal=P3' "$LOG")

echo "log=$LOG"
echo "vehicles_added=$add"
echo "exited=$ex  parked=$pk  completed=$tot  threshold=$THRESH"
echo "no_slot_warnings=$noslot"
echo "tracebacks=$tb"
echo "entry_portal: P1=$ep1 P4=$ep4"
echo "exit_portal:  P1=$xp1 P2=$xp2 P3=$xp3"
if [ "$tot" -ge "$THRESH" ]; then echo "VERDICT=PASS"; else echo "VERDICT=FAIL"; fi
