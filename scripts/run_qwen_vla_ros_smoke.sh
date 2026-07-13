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
SPOT_INDEX="${PARKSIM_VLA_SMOKE_SPOT_INDEX:-7}"
BACKGROUND_MODE="${PARKSIM_VLA_SMOKE_BACKGROUND_MODE:-rule_random}"
QWEN_TIMEOUT="${PARKSIM_VLA_SMOKE_QWEN_TIMEOUT:-60.0}"
SIMULATION_STEP="${PARKSIM_VLA_ROS_SMOKE_SIMULATION_STEP:-0.1}"
SIMULATION_SPEEDUP="${PARKSIM_VLA_ROS_SMOKE_SIMULATION_SPEEDUP:-5.0}"
VEHICLE_LOG_DIR="$(mktemp -d "$LOG_DIR/parksim_qwen_vla_ros_smoke_vehicle.XXXXXX")"
DECISION_LOG="$VEHICLE_LOG_DIR/qwen_vla_decisions.jsonl"

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
  -p timer_period:="$SIMULATION_STEP" \
  -p simulation_speedup:="$SIMULATION_SPEEDUP" \
  -p spawn_qwen_ego:=true \
  -p qwen_ego_spawn_time:=0.5 \
  -p qwen_ego_spot_index:="$SPOT_INDEX" \
  -p qwen_endpoint:="http://127.0.0.1:$PORT/v1/chat/completions" \
  -p qwen_timeout:="$QWEN_TIMEOUT" \
  -p fleet_coordinator_enabled:=true \
  -p fleet_run_id:="$RUN_ID" \
  -p log_path:="$VEHICLE_LOG_DIR" \
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

PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 - "$VEHICLE_LOG_DIR" "$SIMULATION_STEP" "$SIMULATION_SPEEDUP" <<'PYTIMING'
import json
import sys
from pathlib import Path

from parksim.vla.safety_metrics import collect_system_traffic_metrics

log_dir = Path(sys.argv[1])
expected_step = float(sys.argv[2])
expected_speedup = float(sys.argv[3])
traces = sorted(log_dir.glob("vehicle_*_trace.jsonl"))
if not traces:
    raise SystemExit("accelerated ROS smoke produced no vehicle traces")
first = json.loads(next(line for line in traces[0].read_text().splitlines() if line.strip()))
if abs(float(first.get("simulation_step_seconds", 0.0)) - expected_step) > 1e-9:
    raise SystemExit("trace simulation step mismatch: %s" % first)
if abs(float(first.get("simulation_speedup", 0.0)) - expected_speedup) > 1e-9:
    raise SystemExit("trace speedup mismatch: %s" % first)
metrics = collect_system_traffic_metrics(log_dir)
if metrics.get("trace_sim_step_gap_count") != 0:
    raise SystemExit("accelerated ROS smoke skipped simulator steps: %s" % metrics)
if metrics.get("trace_integrity_ok") is not True:
    raise SystemExit("accelerated ROS smoke failed trace integrity: %s" % metrics)
print("parksim accelerated timing smoke ok")
print("simulation_step_seconds=%s" % expected_step)
print("simulation_speedup=%s" % expected_speedup)
print("vehicle_log_dir=%s" % log_dir)
PYTIMING

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
if record.get("simulation_time_policy") != "decision_then_advance":
    raise SystemExit("fleet epoch does not use decision_then_advance semantics")
if record.get("wall_clock_latency_in_performance_metrics") is not False:
    raise SystemExit("wall-clock latency must be excluded from performance metrics")
if record.get("decision_barrier_status") not in ("applied", "no_action_required"):
    raise SystemExit("fleet decision barrier did not complete: %s" % record.get("decision_barrier_status"))
expected_ack = {int(value) for value in record.get("decision_ack_expected_vehicle_ids", [])}
received_ack = {int(value) for value in record.get("decision_ack_received_vehicle_ids", [])}
if not expected_ack or expected_ack != received_ack:
    raise SystemExit("fleet decision ACK coverage mismatch: expected=%s received=%s" % (
        sorted(expected_ack), sorted(received_ack)))
if abs(float(record.get("decision_ack_coverage_rate", 0.0)) - 1.0) > 1e-9:
    raise SystemExit("fleet decision ACK coverage is not 100%")
if record.get("decision_ack_failed_vehicle_ids"):
    raise SystemExit("fleet decision ACK contains failed vehicles: %s" % record["decision_ack_failed_vehicle_ids"])
frozen_time = float(record.get("sim_time"))
acknowledgements = record.get("decision_acknowledgements") or []
if len(acknowledgements) != len(expected_ack):
    raise SystemExit("fleet decision ACK payload count mismatch")
for ack in acknowledgements:
    if not ack.get("applied"):
        raise SystemExit("fleet decision was not applied: %s" % ack)
    if abs(float(ack.get("decision_sim_time")) - frozen_time) > 1e-6:
        raise SystemExit("decision ACK is not aligned with frozen simulation time: %s" % ack)
    if abs(float(ack.get("vehicle_sim_time")) - frozen_time) > 1e-6:
        raise SystemExit("vehicle advanced while Qwen decision was pending: %s" % ack)
print("parksim centralized fleet epoch smoke ok")
print("fleet_epoch_id=%s" % record.get("epoch_id"))
print("fleet_sim_time=%s" % record.get("sim_time"))
print("fleet_decision_ack_coverage=%s" % record.get("decision_ack_coverage_rate"))
print("fleet_log=%s" % fleet_log)
print("sim_log=%s" % sim_log)
PYFLEET
