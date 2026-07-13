#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/training_guard.sh"
parksim_guard_active_training "ParkSim simulation-speedup equivalence smoke"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_ROOT="${PARKSIM_SPEEDUP_SMOKE_OUT_DIR:-$ROOT/experiments/qwen_vla_benchmark/speedup_equivalence_$STAMP}"
SIMULATION_STEP="${PARKSIM_SPEEDUP_SMOKE_SIMULATION_STEP:-0.1}"
ACCELERATED_SPEEDUP="${PARKSIM_SPEEDUP_SMOKE_ACCELERATED_SPEEDUP:-5.0}"
DURATION="${PARKSIM_SPEEDUP_SMOKE_DURATION:-30s}"

run_case() {
  local label="$1"
  local speedup="$2"
  env \
    PARKSIM_BENCH_OUT_DIR="$OUT_ROOT/$label" \
    PARKSIM_BENCH_AGENTS=rule_based \
    PARKSIM_BENCH_SEEDS=0 \
    PARKSIM_BENCH_BACKGROUND_MODES=rule_random \
    PARKSIM_BENCH_DURATION="$DURATION" \
    PARKSIM_BENCH_SIMULATION_STEP="$SIMULATION_STEP" \
    PARKSIM_BENCH_SIMULATION_SPEEDUP="$speedup" \
    PARKSIM_BENCH_WALL_TIMEOUT=120s \
    PARKSIM_BENCH_TRAFFIC_FLOW_MODE=human_mixed_long_horizon \
    PARKSIM_BENCH_SPAWN_ENTERING=0 \
    PARKSIM_BENCH_SPAWN_EXITING=0 \
    PARKSIM_BENCH_AV_SPAWN_ENTERING=0 \
    PARKSIM_BENCH_AV_SPAWN_EXITING=0 \
    PARKSIM_BENCH_RESTORE_OBSTACLES_AS_EXIT_VEHICLES=false \
    PARKSIM_BENCH_EARLY_STOP=0 \
    PARKSIM_BENCH_TRAFFIC_COMPLETION_STOP=0 \
    PARKSIM_BENCH_TRAFFIC_HORIZON_STOP=1 \
    PARKSIM_BENCH_TRAFFIC_HORIZON_OVERRUN_SECONDS=0 \
    PARKSIM_BENCH_PROTECT_ACTIVE_TRAINING=1 \
    PARKSIM_BENCH_ALLOW_DURING_TRAINING=0 \
    "$ROOT/scripts/run_vla_benchmark.sh"
}

run_case baseline_1x 1.0
run_case accelerated "${ACCELERATED_SPEEDUP}"

export PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}"
python3 -m parksim.vla.speedup_equivalence \
  "$OUT_ROOT/baseline_1x" "$OUT_ROOT/accelerated" \
  --simulation-step "$SIMULATION_STEP" \
  --accelerated-speedup "$ACCELERATED_SPEEDUP" \
  --output "$OUT_ROOT/equivalence_report.json"

echo "speedup_equivalence_out=$OUT_ROOT"
