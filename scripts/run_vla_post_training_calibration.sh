#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/training_guard.sh"

TRAINING_REGEX="${PARKSIM_POST_TRAINING_PROCESS_REGEX:-main_MAPPO|main_MAHAN|main_HAN|main_IPPO}"
POLL_SECONDS="${PARKSIM_POST_TRAINING_POLL_SECONDS:-60}"
STABLE_POLLS="${PARKSIM_POST_TRAINING_STABLE_POLLS:-3}"
REQUIRE_GPU_IDLE="${PARKSIM_POST_TRAINING_REQUIRE_GPU_IDLE:-1}"
DRY_RUN="${PARKSIM_POST_TRAINING_DRY_RUN:-0}"
STAMP="$(date +%Y%m%d_%H%M%S)"
STATE_DIR="${PARKSIM_POST_TRAINING_STATE_DIR:-$ROOT/experiments/qwen_vla_jobs/post_training_calibration_$STAMP}"
CALIBRATION_OUT_DIR="${PARKSIM_CALIBRATION_OUT_DIR:-$ROOT/experiments/qwen_vla_paper_suite/post_training_calibration_$STAMP}"
STATE_FILE="$STATE_DIR/status.json"
mkdir -p "$STATE_DIR"

exec 9>"$STATE_DIR/job.lock"
if ! flock -n 9; then
  echo "Another post-training calibration job owns $STATE_DIR/job.lock" >&2
  exit 73
fi

phase="initializing"
completed=0
write_status() {
  local status="$1"
  local detail="${2:-}"
  python3 - "$STATE_FILE" "$status" "$phase" "$detail" "$CALIBRATION_OUT_DIR" <<'PYSTATUS'
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
path, status, phase, detail, calibration_out_dir = sys.argv[1:]
try:
    commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
except Exception:
    commit = "unknown"
payload = {
    "status": status,
    "phase": phase,
    "detail": detail,
    "updated_at": datetime.now(timezone.utc).isoformat(),
    "pid": os.getppid(),
    "git_commit": commit,
    "calibration_out_dir": calibration_out_dir,
}
Path(path).write_text(json.dumps(payload, indent=2) + "\n")
PYSTATUS
}

on_exit() {
  local rc="$?"
  if [[ "$completed" != "1" ]]; then
    write_status failed "exit_status=$rc"
  fi
}
trap on_exit EXIT

gpu_compute_pids() {
  if [[ "$REQUIRE_GPU_IDLE" != "1" ]] || ! command -v nvidia-smi >/dev/null 2>&1; then
    return 0
  fi
  nvidia-smi --query-compute-apps=pid --format=csv,noheader,nounits 2>/dev/null \
    | sed '/^[[:space:]]*$/d'
}

wait_for_idle() {
  phase="waiting_for_training_and_gpu_idle"
  local stable=0
  while (( stable < STABLE_POLLS )); do
    local training_matches gpu_matches
    training_matches="$(pgrep -af "$TRAINING_REGEX" || true)"
    gpu_matches="$(gpu_compute_pids || true)"
    if [[ -z "$training_matches" && -z "$gpu_matches" ]]; then
      stable=$((stable + 1))
      write_status waiting "idle_stability=$stable/$STABLE_POLLS"
    else
      stable=0
      write_status waiting "training_or_gpu_busy"
    fi
    if (( stable < STABLE_POLLS )); then
      sleep "$POLL_SECONDS"
    fi
  done
}

run_phase() {
  phase="$1"
  shift
  write_status running
  PARKSIM_TRAINING_PROCESS_REGEX="$TRAINING_REGEX" parksim_guard_active_training "$phase"
  "$@"
}

write_status waiting
wait_for_idle

if [[ "$DRY_RUN" == "1" ]]; then
  phase="dry_run_complete"
  write_status complete "would run smoke, ROS build, ROS barrier smoke, and 3600 s calibration"
  completed=1
  echo "post_training_calibration_dry_run_state=$STATE_FILE"
  exit 0
fi

run_phase offline_smoke "$ROOT/scripts/run_vla_smoke.sh"
run_phase ros_build "$ROOT/scripts/build_parksim_ros.sh" --symlink-install
run_phase ros_synchronous_barrier_smoke "$ROOT/scripts/run_qwen_vla_ros_smoke.sh"
wait_for_idle
run_phase trc_3600s_calibration env \
  PARKSIM_CALIBRATION_OUT_DIR="$CALIBRATION_OUT_DIR" \
  "$ROOT/scripts/run_vla_trc_calibration.sh"

phase="complete"
write_status complete
completed=1
echo "post_training_calibration_state=$STATE_FILE"
echo "post_training_calibration_out_dir=$CALIBRATION_OUT_DIR"
