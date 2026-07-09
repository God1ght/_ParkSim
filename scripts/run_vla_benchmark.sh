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
TRAFFIC_FLOW_MODE="${PARKSIM_BENCH_TRAFFIC_FLOW_MODE:-legacy}"
LONG_HORIZON_DURATION="${PARKSIM_BENCH_LONG_HORIZON_DURATION:-${DURATION%s}}"
RESTORE_OBSTACLES_AS_EXIT_VEHICLES="${PARKSIM_BENCH_RESTORE_OBSTACLES_AS_EXIT_VEHICLES:-false}"
STATIC_OBSTACLE_EXIT_FRACTION="${PARKSIM_BENCH_STATIC_OBSTACLE_EXIT_FRACTION:-0.35}"
STATIC_OBSTACLE_EXIT_MAX="${PARKSIM_BENCH_STATIC_OBSTACLE_EXIT_MAX:-40}"
STATIC_OBSTACLE_EXIT_START_TIME="${PARKSIM_BENCH_STATIC_OBSTACLE_EXIT_START_TIME:-5.0}"
LONG_HORIZON_ENTER_INTERVAL_MEAN="${PARKSIM_BENCH_LONG_HORIZON_ENTER_INTERVAL_MEAN:-10.0}"
LONG_HORIZON_EXIT_INTERVAL_MEAN="${PARKSIM_BENCH_LONG_HORIZON_EXIT_INTERVAL_MEAN:-14.0}"
HUMAN_INTENT_HIDDEN_FRACTION="${PARKSIM_BENCH_HUMAN_INTENT_HIDDEN_FRACTION:-0.75}"
MAX_CONCURRENT_BACKGROUND_VEHICLES="${PARKSIM_BENCH_MAX_CONCURRENT_BACKGROUND_VEHICLES:-80}"
DELAYED_SPAWN_RETRY_SECONDS="${PARKSIM_BENCH_DELAYED_SPAWN_RETRY_SECONDS:-2.0}"
EXIT_SPOT_REUSE_DELAY="${PARKSIM_BENCH_EXIT_SPOT_REUSE_DELAY:-20.0}"
QWEN_PERIODIC_REPLAN="${PARKSIM_BENCH_QWEN_PERIODIC_REPLAN:-false}"
CONTROLLED_EGO_BLOCKS_ENTRANCE="${PARKSIM_BENCH_CONTROLLED_EGO_BLOCKS_ENTRANCE:-true}"
QWEN_MODE="${PARKSIM_BENCH_QWEN_MODE:-mock}"
QWEN_PORT="${PARKSIM_BENCH_QWEN_PORT:-18087}"
QWEN_ENDPOINT="${PARKSIM_BENCH_QWEN_ENDPOINT:-}"
QWEN_TIMEOUT="${PARKSIM_BENCH_QWEN_TIMEOUT:-120.0}"
QWEN_STARTUP_TIMEOUT="${PARKSIM_BENCH_QWEN_STARTUP_TIMEOUT:-900}"
EARLY_STOP="${PARKSIM_BENCH_EARLY_STOP:-1}"
EARLY_STOP_POLL_SECONDS="${PARKSIM_BENCH_EARLY_STOP_POLL_SECONDS:-1}"
EARLY_STOP_GRACE_SECONDS="${PARKSIM_BENCH_EARLY_STOP_GRACE_SECONDS:-2}"
CLEAR_OUT_DIR="${PARKSIM_BENCH_CLEAR_OUT_DIR:-1}"

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

qwen_pid=""
GIT_COMMIT="$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git -C "$ROOT" branch --show-current 2>/dev/null || echo unknown)"
export ROOT GIT_COMMIT GIT_BRANCH DURATION AGENTS SEEDS BACKGROUND_MODES SPOT_INDEX SPAWN_ENTERING SPAWN_EXITING TRAFFIC_FLOW_MODE LONG_HORIZON_DURATION RESTORE_OBSTACLES_AS_EXIT_VEHICLES STATIC_OBSTACLE_EXIT_FRACTION STATIC_OBSTACLE_EXIT_MAX STATIC_OBSTACLE_EXIT_START_TIME LONG_HORIZON_ENTER_INTERVAL_MEAN LONG_HORIZON_EXIT_INTERVAL_MEAN HUMAN_INTENT_HIDDEN_FRACTION MAX_CONCURRENT_BACKGROUND_VEHICLES DELAYED_SPAWN_RETRY_SECONDS EXIT_SPOT_REUSE_DELAY QWEN_PERIODIC_REPLAN CONTROLLED_EGO_BLOCKS_ENTRANCE QWEN_MODE QWEN_ENDPOINT QWEN_TIMEOUT EARLY_STOP EARLY_STOP_POLL_SECONDS EARLY_STOP_GRACE_SECONDS CLEAR_OUT_DIR

prepare_out_dir() {
  mkdir -p "$OUT_DIR"
  local out_abs
  out_abs="$(realpath -m "$OUT_DIR")"
  if [[ "$CLEAR_OUT_DIR" == "1" ]]; then
    case "$out_abs" in
      "$ROOT"/experiments/qwen_vla_benchmark/*|"$ROOT"/experiments/qwen_vla_long_horizon/*|"$ROOT"/experiments/qwen_vla_paper_suite/*/benchmarks/*)
        rm -rf "$OUT_DIR/episodes"
        rm -f \
          "$OUT_DIR/episodes.jsonl" \
          "$OUT_DIR/metrics.csv" \
          "$OUT_DIR/metrics.json" \
          "$OUT_DIR/summary.md" \
          "$OUT_DIR/summary.json" \
          "$OUT_DIR/preference_dataset.jsonl" \
          "$OUT_DIR/manifest.json" \
          "$OUT_DIR/validation.json" \
          "$OUT_DIR/decision_audit.json" \
          "$OUT_DIR/decision_audit_failures.jsonl" \
          "$OUT_DIR/run_config.json" \
          "$OUT_DIR/qwen_health.json"
        ;;
      *)
        echo "Refusing to clear benchmark output outside project experiments: $OUT_DIR" >&2
        exit 1
        ;;
    esac
  fi
  mkdir -p "$OUT_DIR/episodes"
  : > "$OUT_DIR/episodes.jsonl"
}
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
    "traffic_flow_mode": os.environ.get("TRAFFIC_FLOW_MODE", ""),
    "long_horizon_duration": os.environ.get("LONG_HORIZON_DURATION", ""),
    "restore_obstacles_as_exit_vehicles": os.environ.get("RESTORE_OBSTACLES_AS_EXIT_VEHICLES", ""),
    "static_obstacle_exit_fraction": os.environ.get("STATIC_OBSTACLE_EXIT_FRACTION", ""),
    "static_obstacle_exit_max": os.environ.get("STATIC_OBSTACLE_EXIT_MAX", ""),
    "long_horizon_enter_interval_mean": os.environ.get("LONG_HORIZON_ENTER_INTERVAL_MEAN", ""),
    "long_horizon_exit_interval_mean": os.environ.get("LONG_HORIZON_EXIT_INTERVAL_MEAN", ""),
    "human_intent_hidden_fraction": os.environ.get("HUMAN_INTENT_HIDDEN_FRACTION", ""),
    "max_concurrent_background_vehicles": os.environ.get("MAX_CONCURRENT_BACKGROUND_VEHICLES", ""),
    "exit_spot_reuse_delay": os.environ.get("EXIT_SPOT_REUSE_DELAY", ""),
    "qwen_periodic_replan": os.environ.get("QWEN_PERIODIC_REPLAN", ""),
    "controlled_ego_blocks_entrance": os.environ.get("CONTROLLED_EGO_BLOCKS_ENTRANCE", ""),
    "qwen_mode": os.environ.get("QWEN_MODE", ""),
    "qwen_endpoint": os.environ.get("QWEN_ENDPOINT", ""),
    "qwen_timeout": os.environ.get("QWEN_TIMEOUT", ""),
    "early_stop": os.environ.get("EARLY_STOP", ""),
    "early_stop_poll_seconds": os.environ.get("EARLY_STOP_POLL_SECONDS", ""),
    "early_stop_grace_seconds": os.environ.get("EARLY_STOP_GRACE_SECONDS", ""),
    "clear_out_dir": os.environ.get("CLEAR_OUT_DIR", ""),
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
    if python3 -c 'import json,sys; from urllib import request; url=sys.argv[1]+"/healthz"; expected=sys.argv[2]; resp=request.urlopen(url, timeout=1.0); payload=json.loads(resp.read().decode("utf-8")); print(json.dumps(payload)); raise SystemExit(0 if payload.get("ok") and (expected == "any" or str(payload.get("mock")).lower() == expected) else 1)' "$base_url" "$expected_mock" > "$OUT_DIR/qwen_health.json" 2>/dev/null; then
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
  local early_stop_triggered="${10:-0}"
  python3 - "$file" "$scenario_id" "$scenario_dir" "$agent" "$seed" "$background_mode" "$SPOT_INDEX" "$SPAWN_ENTERING" "$SPAWN_EXITING" "$run_dir" "$log_dir" "$sim_log" "$EARLY_STOP" "$early_stop_triggered" "$DURATION" <<'EPISODE_JSON'
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
    "early_stop_enabled": sys.argv[13] == "1",
    "early_stop_triggered": sys.argv[14] == "1",
    "duration_limit": sys.argv[15],
}
with open(path, "a") as f:
    f.write(json.dumps(record) + "\n")
EPISODE_JSON
}

controlled_ego_done() {
  local log_dir="$1"
  python3 - "$log_dir" <<'EARLY_STOP_CHECK'
import json
import sys
from pathlib import Path

log_dir = Path(sys.argv[1])
completed_fallback = False
for summary_path in sorted(log_dir.glob("vehicle_*_summary.json")):
    try:
        payload = json.loads(summary_path.read_text())
    except Exception:
        continue
    if payload.get("completed") is not True:
        continue
    if payload.get("is_controlled_ego") is True:
        raise SystemExit(0)
    if payload.get("vehicle_id") == 1:
        completed_fallback = True
if completed_fallback:
    raise SystemExit(0)
raise SystemExit(1)
EARLY_STOP_CHECK
}

wait_for_early_stop() {
  local sim_pid="$1"
  local log_dir="$2"
  if [[ "$EARLY_STOP" != "1" ]]; then
    return 1
  fi
  while kill -0 "$sim_pid" 2>/dev/null; do
    if controlled_ego_done "$log_dir" >/dev/null 2>&1; then
      echo "early_stop controlled ego completed -> $log_dir"
      sleep "$EARLY_STOP_GRACE_SECONDS"
      cleanup_ros_processes
      return 0
    fi
    sleep "$EARLY_STOP_POLL_SECONDS"
  done
  return 1
}

write_early_stop_marker() {
  local run_dir="$1"
  local triggered="$2"
  python3 - "$run_dir/early_stop.json" "$EARLY_STOP" "$triggered" "$EARLY_STOP_POLL_SECONDS" "$EARLY_STOP_GRACE_SECONDS" <<'EARLY_STOP_MARKER'
import json
import sys
payload = {
    "enabled": sys.argv[2] == "1",
    "triggered": sys.argv[3] == "1",
    "poll_seconds": float(sys.argv[4]),
    "grace_seconds": float(sys.argv[5]),
}
with open(sys.argv[1], "w") as f:
    json.dump(payload, f, indent=2)
EARLY_STOP_MARKER
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
  local qwen_endpoint_param="${QWEN_ENDPOINT:-http://127.0.0.1:0/v1/chat/completions}"
  mkdir -p "$log_dir"
  echo "running scenario=$scenario_id agent=$agent -> $run_dir"
  cleanup_ros_processes
  set +e
  local early_stop_triggered=0
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
    -p traffic_flow_mode:="$TRAFFIC_FLOW_MODE" \
    -p long_horizon_duration:="$LONG_HORIZON_DURATION" \
    -p restore_obstacles_as_exit_vehicles:="$RESTORE_OBSTACLES_AS_EXIT_VEHICLES" \
    -p static_obstacle_exit_fraction:="$STATIC_OBSTACLE_EXIT_FRACTION" \
    -p static_obstacle_exit_max:="$STATIC_OBSTACLE_EXIT_MAX" \
    -p static_obstacle_exit_start_time:="$STATIC_OBSTACLE_EXIT_START_TIME" \
    -p long_horizon_enter_interval_mean:="$LONG_HORIZON_ENTER_INTERVAL_MEAN" \
    -p long_horizon_exit_interval_mean:="$LONG_HORIZON_EXIT_INTERVAL_MEAN" \
    -p human_intent_hidden_fraction:="$HUMAN_INTENT_HIDDEN_FRACTION" \
    -p max_concurrent_background_vehicles:="$MAX_CONCURRENT_BACKGROUND_VEHICLES" \
    -p delayed_spawn_retry_seconds:="$DELAYED_SPAWN_RETRY_SECONDS" \
    -p exit_spot_reuse_delay:="$EXIT_SPOT_REUSE_DELAY" \
    -p qwen_periodic_replan:="$QWEN_PERIODIC_REPLAN" \
    -p log_path:="$log_dir" \
    -p qwen_endpoint:="$qwen_endpoint_param" \
    -p qwen_timeout:="$QWEN_TIMEOUT" \
    > "$sim_log" 2>&1 &
  local sim_pid="$!"
  if wait_for_early_stop "$sim_pid" "$log_dir"; then
    early_stop_triggered=1
  fi
  wait "$sim_pid"
  local status="$?"
  set -e
  cleanup_ros_processes
  write_early_stop_marker "$run_dir" "$early_stop_triggered"
  if [[ "$early_stop_triggered" == "1" ]]; then
    status=0
  fi
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
  append_episode "$OUT_DIR/episodes.jsonl" "$scenario_id" "$scenario_dir" "$agent" "$seed" "$background_mode" "$run_dir" "$log_dir" "$sim_log" "$early_stop_triggered"
}

prepare_out_dir
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
