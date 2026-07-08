#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if command -v conda >/dev/null 2>&1; then
  # ROS Foxy on this host needs system Python packages such as catkin_pkg.
  # An active conda Python causes ament CMake package parsing to fail.
  set +u
  source "$(conda info --base)/etc/profile.d/conda.sh"
  while [ -n "${CONDA_PREFIX:-}" ]; do
    conda deactivate || break
  done
  set -u
fi

set +u
source /opt/ros/foxy/setup.bash
set -u
cd "$ROOT/workspace"
exec colcon build --packages-select parksim "$@"
