#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$ROOT/workspace/install/setup.bash}"
DLP_ROOT="${PARKSIM_DLP_ROOT:-/media/step/data/Parking_Yccc7/_dlp_dataset}"
OUT_DIR="${PARKSIM_BENCH_OUT_DIR:-$ROOT/experiments/qwen_vla_benchmark/$(date +%Y%m%d_%H%M%S)}"
DURATION="${PARKSIM_BENCH_DURATION:-90s}"
AGENTS="${PARKSIM_BENCH_AGENTS:-rule_based greedy_nearest greedy_shortest_path risk_aware_rule qwen_vla}"
SEEDS="${PARKSIM_BENCH_SEEDS:-0 1 2}"
BACKGROUND_MODES="${PARKSIM_BENCH_BACKGROUND_MODES:-rule_random}"
SPOT_INDEX="${PARKSIM_BENCH_SPOT_INDEX:-7}"
SPAWN_TIME="${PARKSIM_BENCH_SPAWN_TIME:-0.5}"
SPAWN_ENTERING="${PARKSIM_BENCH_SPAWN_ENTERING:-0}"
SPAWN_EXITING="${PARKSIM_BENCH_SPAWN_EXITING:-0}"
CONTROLLED_EGO_BLOCKS_ENTRANCE="${PARKSIM_BENCH_CONTROLLED_EGO_BLOCKS_ENTRANCE:-true}"
QWEN_MODE="${PARKSIM_BENCH_QWEN_MODE:-mock}"
QWEN_PORT="${PARKSIM_BENCH_QWEN_PORT:-18087}"
QWEN_ENDPOINT="${PARKSIM_BENCH_QWEN_ENDPOINT:-}"
QWEN_TIMEOUT="${PARKSIM_BENCH_QWEN_TIMEOUT:-120.0}"
QWEN_STARTUP_TIMEOUT="${PARKSIM_BENCH_QWEN_STARTUP_TIMEOUT:-900}"

# ROS Foxy on Ubuntu 20.04 is built against system Python 3.8.
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

mkdir -p "$OUT_DIR/episodes"
: > "$OUT_DIR/episodes.jsonl"
qwen_pid=""
GIT_COMMIT="$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git -C "$ROOT" branch --show-current 2>/dev/null || echo unknown)"
export ROOT GIT_COMMIT GIT_BRANCH DURATION AGENTS SEEDS BACKGROUND_MODES SPOT_INDEX SPAWN_ENTERING SPAWN_EXITING CONTROLLED_EGO_BLOCKS_ENTRANCE QWEN_MODE QWEN_ENDPOINT
write_run_config() {
  python3 - "$OUT_DIR/run_config.json" <<'RUN_CONFIG_JSON'
import json
import os
import sys
payload = {
    "root": os.environ.get("ROOT", ""),
    "git_commit": os.environ.get("GIT_COMMIT", ""),
    "git_branch": os.environ.get("GIT_BRANCH", ""),
    "duration": os.environ.get("DURATION", ""),
    "agents": os.environ.get("AGENTS", "").split(),
    "seeds": os.environ.get("SEEDS", "").split(),
    "background_modes": os.environ.get("BACKGROUND_MODES", "").split(),
    "spot_index": os.environ.get("SPOT_INDEX", ""),
    "spawn_entering": os.environ.get("SPAWN_ENTERING", ""),
    "spawn_exiting": os.environ.get("SPAWN_EXITING", ""),
    "controlled_ego_blocks_entrance": os.environ.get("CONTROLLED_EGO_BLOCKS_ENTRANCE", ""),
    "qwen_mode": os.environ.get("QWEN_MODE", ""),
    "qwen_endpoint": os.environ.get("QWEN_ENDPOINT", ""),
}
with open(sys.argv[1], "w") as f:
    json.dump(payload, f, indent=2)
RUN_CONFIG_JSON

}

cleanup_ros_processes() {
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/simulator_node.py" 2>/dev/null || true
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/vehicle_node.py" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/simulator_node.py" 2>/dev/null || true
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/vehicle_node.py" 2>/dev/null || true
}

cleanup() {
  cleanup_ros_processes
  if [[ -n "$qwen_pid" ]] && kill -0 "$qwen_pid" 2>/dev/null; then
    kill "$qwen_pid" 2>/dev/null || true
    wait "$qwen_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

needs_qwen() {
  for agent in $AGENTS; do
    if [[ "$agent" == "qwen_vla" ]]; then
      return 0
    fi
  done
  return 1
}

wait_for_qwen() {
  local endpoint="$1"
  local expected_mock="$2"
  local base_url="${endpoint%/v1/chat/completions}"
  local ready=0
  for _ in $(seq 1 "$QWEN_STARTUP_TIMEOUT"); do
    if python3 -c 'import json,sys; from urllib import request; url=sys.argv[1]+"/healthz"; expected=sys.argv[2]; resp=request.urlopen(url, timeout=1.0); payload=json.loads(resp.read().decode("utf-8")); raise SystemExit(0 if payload.get("ok") and (expected == "any" or str(payload.get("mock")).lower() == expected) else 1)' "$base_url" "$expected_mock" > "$OUT_DIR/qwen_health.json" 2>/dev/null; then
      ready=1
      break
    fi
    if [[ -n "$qwen_pid" ]] && ! kill -0 "$qwen_pid" 2>/dev/null; then
      echo "Qwen service exited early" >&2
      tail -n 120 "$OUT_DIR/qwen_service.log" >&2 || true
      exit 1
    fi
    sleep 1
  done
  if [[ "$ready" != "1" ]]; then
    echo "Qwen service did not become healthy: $endpoint" >&2
    tail -n 120 "$OUT_DIR/qwen_service.log" >&2 || true
    exit 1
  fi
}

start_qwen_if_needed() {
  if ! needs_qwen; then
    return
  fi
  if [[ -n "$QWEN_ENDPOINT" ]]; then
    wait_for_qwen "$QWEN_ENDPOINT" "any"
    return
  fi
  QWEN_ENDPOINT="http://127.0.0.1:$QWEN_PORT/v1/chat/completions"
  case "$QWEN_MODE" in
    mock)
      QWEN_VLA_PORT="$QWEN_PORT" "$ROOT/scripts/run_qwen_vla_service.sh" --mock > "$OUT_DIR/qwen_service.log" 2>&1 &
      qwen_pid="$!"
      wait_for_qwen "$QWEN_ENDPOINT" "true"
      ;;
    real)
      QWEN_VLA_PORT="$QWEN_PORT" "$ROOT/scripts/run_qwen_vla_service.sh" > "$OUT_DIR/qwen_service.log" 2>&1 &
      qwen_pid="$!"
      wait_for_qwen "$QWEN_ENDPOINT" "false"
      ;;
    external)
      echo "PARKSIM_BENCH_QWEN_MODE=external requires PARKSIM_BENCH_QWEN_ENDPOINT" >&2
      exit 1
      ;;
    *)
      echo "Unsupported PARKSIM_BENCH_QWEN_MODE: $QWEN_MODE" >&2
      exit 1
      ;;
  esac
}

append_episode() {
  local file="$1"
  local scenario_id="$2"
  local scenario_dir="$3"
  local agent="$4"
  local seed="$5"
  local background_mode="$6"
  local run_dir="$7"
  local log_dir="$8"
  local sim_log="$9"
  python3 - "$file" "$scenario_id" "$scenario_dir" "$agent" "$seed" "$background_mode" "$SPOT_INDEX" "$SPAWN_ENTERING" "$SPAWN_EXITING" "$run_dir" "$log_dir" "$sim_log" <<'EPISODE_JSON'
import json
import sys
path = sys.argv[1]
record = {
    "scenario_id": sys.argv[2],
    "scenario_dir": sys.argv[3],
    "agent_type": sys.argv[4],
    "seed": int(sys.argv[5]),
    "background_mode": sys.argv[6],
    "spot_index": int(sys.argv[7]),
    "spawn_entering": int(sys.argv[8]),
    "spawn_exiting": int(sys.argv[9]),
    "run_dir": sys.argv[10],
    "log_dir": sys.argv[11],
    "simulator_log": sys.argv[12],
}
with open(path, "a") as f:
    f.write(json.dumps(record) + "\n")
EPISODE_JSON
}

run_episode() {
  local agent="$1"
  local seed="$2"
  local background_mode="$3"
  local scenario_id="${background_mode}_seed${seed}_spot${SPOT_INDEX}_enter${SPAWN_ENTERING}_exit${SPAWN_EXITING}"
  local scenario_dir="$OUT_DIR/episodes/$scenario_id"
  local run_dir="$scenario_dir/$agent"
  local log_dir="$run_dir/logs"
  local sim_log="$run_dir/simulator.log"
  mkdir -p "$log_dir"
  echo "running scenario=$scenario_id agent=$agent -> $run_dir"
  cleanup_ros_processes
  set +e
  PYTHONPATH="$DLP_ROOT${PYTHONPATH:+:$PYTHONPATH}" timeout "$DURATION" ros2 run parksim simulator_node.py --ros-args \
    -p spawn_controlled_ego:=true \
    -p controlled_ego_agent_type:="$agent" \
    -p controlled_ego_spawn_time:="$SPAWN_TIME" \
    -p controlled_ego_spot_index:="$SPOT_INDEX" \
    -p controlled_ego_blocks_entrance:="$CONTROLLED_EGO_BLOCKS_ENTRANCE" \
    -p random_seed:="$seed" \
    -p background_mode:="$background_mode" \
    -p spawn_entering:="$SPAWN_ENTERING" \
    -p spawn_exiting:="$SPAWN_EXITING" \
    -p log_path:="$log_dir" \
    -p qwen_endpoint:="$QWEN_ENDPOINT" \
    -p qwen_timeout:="$QWEN_TIMEOUT" \
    > "$sim_log" 2>&1
  local status="$?"
  set -e
  cleanup_ros_processes
  if [[ "$status" != "0" && "$status" != "124" ]]; then
    echo "$agent simulator failed with exit status $status" >&2
    tail -n 160 "$sim_log" >&2 || true
    exit "$status"
  fi
  if ! compgen -G "$log_dir/vehicle_*_trace.jsonl" > /dev/null; then
    echo "$agent did not produce a vehicle trace" >&2
    tail -n 160 "$sim_log" >&2 || true
    exit 1
  fi
  append_episode "$OUT_DIR/episodes.jsonl" "$scenario_id" "$scenario_dir" "$agent" "$seed" "$background_mode" "$run_dir" "$log_dir" "$sim_log"
}

start_qwen_if_needed
write_run_config
for background_mode in $BACKGROUND_MODES; do
  for seed in $SEEDS; do
    for agent in $AGENTS; do
      run_episode "$agent" "$seed" "$background_mode"
    done
  done
done

PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.benchmark collect "$OUT_DIR" --root "$ROOT"
expected_agents_csv="$(printf '%s' "$AGENTS" | tr ' ' ',')"
validate_args=("$OUT_DIR" --expected-agents "$expected_agents_csv")
if [[ "${PARKSIM_BENCH_REQUIRE_COMPLETE:-0}" == "1" ]]; then
  validate_args+=(--require-complete)
fi
PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.benchmark validate "${validate_args[@]}"

echo "benchmark_out_dir=$OUT_DIR"
