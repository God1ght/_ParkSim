#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/training_guard.sh"
parksim_guard_active_training "ParkSim Qwen VLA ROS smoke"

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$ROOT/workspace/install/setup.bash}"
DLP_ROOT="${PARKSIM_DLP_ROOT:-/media/step/data/Parking_Yccc7/_dlp_dataset}"
PORT="${QWEN_VLA_SMOKE_PORT:-18085}"
DURATION="${PARKSIM_VLA_ROS_SMOKE_DURATION:-45s}"
LOG_DIR="${PARKSIM_VLA_SMOKE_LOG_DIR:-/tmp}"
QWEN_LOG="$LOG_DIR/qwen_vla_ros_smoke_service.log"
SIM_LOG="$LOG_DIR/parksim_qwen_vla_ros_smoke.log"
FLEET_LOG="$LOG_DIR/parksim_qwen_vla_ros_smoke_fleet.log"
FLEET_EPOCH_LOG="$(mktemp "$LOG_DIR/parksim_qwen_vla_ros_smoke_epochs.XXXXXX.jsonl")"
RUN_ID="ros-smoke-$PORT"
DECISION_LOG="$ROOT/vehicle_log/qwen_vla_decisions.jsonl"
SPOT_INDEX="${PARKSIM_VLA_SMOKE_SPOT_INDEX:-7}"
BACKGROUND_MODE="${PARKSIM_VLA_SMOKE_BACKGROUND_MODE:-rule_random}"
QWEN_TIMEOUT="${PARKSIM_VLA_SMOKE_QWEN_TIMEOUT:-60.0}"

# ROS Foxy on Ubuntu 20.04 is built against the system Python 3.8.
# Keep Conda from shadowing rclpy when this script is launched from a Conda shell.
export PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset PYTHONHOME
unset PYTHONPATH

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "Workspace setup file not found: $WORKSPACE_SETUP. Run ./scripts/build_parksim_ros.sh first." >&2
  exit 1
fi
if [[ ! -d "$DLP_ROOT/dlp" ]]; then
  echo "DLP package not found under $DLP_ROOT. Set PARKSIM_DLP_ROOT or copy DLP data to this host." >&2
  exit 1
fi

set +u
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
set -u

qwen_pid=""
fleet_pid=""
cleanup_ros_processes() {
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/simulator_node.py" 2>/dev/null || true
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/vehicle_node.py" 2>/dev/null || true
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/fleet_coordinator_node.py" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/simulator_node.py" 2>/dev/null || true
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/vehicle_node.py" 2>/dev/null || true
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/fleet_coordinator_node.py" 2>/dev/null || true
}

cleanup() {
  cleanup_ros_processes
  if [[ -n "$qwen_pid" ]] && kill -0 "$qwen_pid" 2>/dev/null; then
    kill "$qwen_pid" 2>/dev/null || true
    wait "$qwen_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

QWEN_VLA_PORT="$PORT" "$ROOT/scripts/run_qwen_vla_service.sh" --mock > "$QWEN_LOG" 2>&1 &
qwen_pid="$!"

ready=0
for _ in $(seq 1 45); do
  if python3 - "$PORT" <<'PYHEALTH' >/dev/null 2>&1
import json
import sys
from urllib import request
port = sys.argv[1]
with request.urlopen(f"http://127.0.0.1:{port}/healthz", timeout=1.0) as resp:
    payload = json.loads(resp.read().decode("utf-8"))
if not payload.get("ok"):
    raise SystemExit(1)
PYHEALTH
  then
    ready=1
    break
  fi
  if ! kill -0 "$qwen_pid" 2>/dev/null; then
    echo "Qwen VLA mock service exited early" >&2
    tail -n 80 "$QWEN_LOG" >&2 || true
    exit 1
  fi
  sleep 1
done
if [[ "$ready" != "1" ]]; then
  echo "Qwen VLA mock service did not become healthy on port $PORT" >&2
  tail -n 80 "$QWEN_LOG" >&2 || true
  exit 1
fi

ros2 run parksim fleet_coordinator_node.py --ros-args \
  -p qwen_endpoint:="http://127.0.0.1:$PORT/v1/chat/completions" \
  -p qwen_timeout:="$QWEN_TIMEOUT" -p run_id:="$RUN_ID" \
  -p decision_log_path:="$FLEET_EPOCH_LOG" > "$FLEET_LOG" 2>&1 &
fleet_pid="$!"
sleep 1
if ! kill -0 "$fleet_pid" 2>/dev/null; then
  echo "Fleet coordinator exited early" >&2
  tail -n 120 "$FLEET_LOG" >&2 || true
  exit 1
fi

export PYTHONPATH="$DLP_ROOT${PYTHONPATH:+:$PYTHONPATH}"
set +e
timeout "$DURATION" ros2 run parksim simulator_node.py --ros-args \
  -p spawn_qwen_ego:=true \
  -p qwen_ego_spawn_time:=0.5 \
  -p qwen_ego_spot_index:="$SPOT_INDEX" \
  -p qwen_endpoint:="http://127.0.0.1:$PORT/v1/chat/completions" \
  -p qwen_timeout:="$QWEN_TIMEOUT" \
  -p fleet_coordinator_enabled:=true \
  -p fleet_run_id:="$RUN_ID" \
  -p background_mode:="$BACKGROUND_MODE" \
  -p spawn_entering:=0 \
  -p spawn_exiting:=0 \
  > "$SIM_LOG" 2>&1
sim_status="$?"
set -e
cleanup_ros_processes

if [[ "$sim_status" != "0" && "$sim_status" != "124" ]]; then
  echo "Simulator smoke failed with exit status $sim_status" >&2
  tail -n 160 "$SIM_LOG" >&2 || true
  exit "$sim_status"
fi

if ! grep -q "Agent Type: qwen_vla" "$SIM_LOG"; then
  echo "qwen_vla vehicle did not start; expected Agent Type log is missing" >&2
  tail -n 160 "$SIM_LOG" >&2 || true
  exit 1
fi

if [[ ! -f "$DECISION_LOG" ]]; then
  echo "Decision log was not created: $DECISION_LOG" >&2
  tail -n 160 "$SIM_LOG" >&2 || true
  exit 1
fi
after_lines="$(wc -l < "$DECISION_LOG")"
if (( after_lines <= 0 )); then
  echo "Decision log contains no records" >&2
  tail -n 160 "$SIM_LOG" >&2 || true
  exit 1
fi

python3 - "$DECISION_LOG" "$SIM_LOG" "$QWEN_LOG" <<'PYCHECK'
import json
import sys
from pathlib import Path
path = Path(sys.argv[1])
sim_log = sys.argv[2]
qwen_log = sys.argv[3]
records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
if not records:
    raise SystemExit("no new decision records")
record = records[-1]
decision = record.get("decision") or {}
applied = record.get("applied_action") or {}
if not decision.get("action_id"):
    raise SystemExit("decision action_id is missing")
if not applied.get("action_id"):
    raise SystemExit("applied action_id is missing")
if decision.get("used_fallback"):
    raise SystemExit("mock qwen fleet response unexpectedly used fallback")
if record.get("shield_reason") != "ok":
    raise SystemExit(f"unexpected shield_reason: {record.get('shield_reason')}")
print("parksim qwen_vla ros smoke ok")
print("decision_action_id=%s" % decision.get("action_id"))
print("applied_action_type=%s" % applied.get("action_type"))
print("sim_log=%s" % sim_log)
print("qwen_log=%s" % qwen_log)
PYCHECK

python3 - "$FLEET_EPOCH_LOG" "$SIM_LOG" "$FLEET_LOG" <<'PYFLEET'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
sim_log = sys.argv[2]
fleet_log = sys.argv[3]
records = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
complete = [record for record in records if record.get("decision_complete")]
if not complete:
    raise SystemExit("no completed centralized fleet epoch")
record = complete[-1]
if record.get("missing_vehicle_ids"):
    raise SystemExit("fleet epoch has missing vehicles: %s" % record["missing_vehicle_ids"])
if len(record.get("expected_vehicle_ids", [])) != len(record.get("collected_vehicle_ids", [])):
    raise SystemExit("fleet epoch is not a complete batch")
if not record.get("fleet_decisions"):
    raise SystemExit("centralized fleet epoch has no decisions")
print("parksim centralized fleet epoch smoke ok")
print("fleet_epoch_id=%s" % record.get("epoch_id"))
print("fleet_sim_time=%s" % record.get("sim_time"))
print("fleet_log=%s" % fleet_log)
print("sim_log=%s" % sim_log)
PYFLEET
