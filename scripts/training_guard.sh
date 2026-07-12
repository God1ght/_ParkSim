#!/usr/bin/env bash

parksim_guard_active_training() {
  local workload="${1:-ParkSim workload}"
  local training_regex="${PARKSIM_TRAINING_PROCESS_REGEX:-main_MAPPO|main_MAHAN|main_HAN|main_IPPO}"
  local matches

  if ! command -v pgrep >/dev/null 2>&1; then
    echo "Cannot verify active training because pgrep is unavailable; refusing to start $workload." >&2
    exit 75
  fi

  matches="$(pgrep -af "$training_regex" || true)"
  if [[ -z "$matches" ]]; then
    return 0
  fi

  echo "Refusing to start $workload while protected training is active." >&2
  echo "$matches" >&2
  echo "Wait for training to finish before running ParkSim smoke workloads." >&2
  exit 75
}
