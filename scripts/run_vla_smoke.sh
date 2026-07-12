#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/training_guard.sh"
parksim_guard_active_training "ParkSim VLA smoke"
cd "$ROOT"
export PYTHONPATH="$PWD/python${PYTHONPATH:+:$PYTHONPATH}"
python3 -m parksim.vla.smoke
python3 -m parksim.vla.integrity_smoke
python3 -m parksim.vla.fleet_smoke
python3 -m parksim.vla.http_smoke
python3 -m parksim.vla.service_smoke
