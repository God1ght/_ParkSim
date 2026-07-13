#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
source "$ROOT/scripts/training_guard.sh"
ACTION="${1:-}"
STAGED_ROOT="${PARKSIM_TRC_STAGED_ROOT:-$ROOT/experiments/qwen_vla_trc_staged/$(date +%Y%m%d)}"
FORMAL_AGENTS="rule_based risk_aware_rule reservation_bundle centralized_min_cost oracle_intent_bundle fleet_min_cost mllm_direct mllm_self_reflect mllm_external_feedback"
DENSITIES="low:300:225:17.1429:23.5714:40.0:55.0 medium:450:338:11.4286:15.7143:26.6667:36.6667 high:600:450:8.5714:11.7857:20.0:27.5 replay:0:0"
DRY_RUN="${PARKSIM_TRC_STAGE_DRY_RUN:-0}"

usage() {
  cat <<'USAGE'
Usage: run_vla_trc_staged_suite.sh {core|mixed_density|human_behavior|replication|finalize}

Stages are intentionally explicit so each CSV result group can be inspected
before the next group starts. All simulation stages use 3600 s episodes and
refuse to run while protected MAPPO/MAHAN/HAN/IPPO processes are active.
USAGE
}

run_stage() {
  local stage="$1"
  local seeds="$2"
  local backgrounds="$3"
  local densities="$4"
  local out_dir="$STAGED_ROOT/stages/$stage"
  if [[ "$DRY_RUN" != "1" ]]; then
    parksim_guard_active_training "ParkSim TR-C stage $stage"
  fi
  mkdir -p "$STAGED_ROOT/stages"
  env \
    PARKSIM_SUITE_OUT_DIR="$out_dir" \
    PARKSIM_SUITE_REPORT_NAME=stage_report \
    PARKSIM_SUITE_AGENTS="$FORMAL_AGENTS" \
    PARKSIM_SUITE_REFERENCE_AGENT=mllm_external_feedback \
    PARKSIM_SUITE_SEEDS="$seeds" \
    PARKSIM_SUITE_BACKGROUND_MODES="$backgrounds" \
    PARKSIM_SUITE_DENSITY_CONFIGS="$densities" \
    PARKSIM_SUITE_DURATION=3600s \
    PARKSIM_SUITE_SIMULATION_STEP="${PARKSIM_TRC_SIMULATION_STEP:-0.1}" \
    PARKSIM_SUITE_SIMULATION_SPEEDUP="${PARKSIM_TRC_SIMULATION_SPEEDUP:-5.0}" \
    PARKSIM_SUITE_QWEN_MODE=real \
    PARKSIM_SUITE_QWEN_TIMEOUT="${PARKSIM_TRC_QWEN_TIMEOUT:-180.0}" \
    PARKSIM_SUITE_FLEET_DECISION_PERIOD="${PARKSIM_TRC_FLEET_DECISION_PERIOD:-30.0}" \
    PARKSIM_SUITE_FLEET_DECISION_ACK_TIMEOUT="${PARKSIM_TRC_FLEET_DECISION_ACK_TIMEOUT:-10.0}" \
    PARKSIM_SUITE_TRAFFIC_FLOW_MODE=human_mixed_long_horizon \
    PARKSIM_SUITE_FLEET_CONTROL_MODE=cloud_av \
    PARKSIM_SUITE_ENTRY_VEHICLE_AGENT_TYPE=__agent__ \
    PARKSIM_SUITE_EXIT_VEHICLE_AGENT_TYPE=__agent__ \
    PARKSIM_SUITE_AV_SPAWN_ENTERING="${PARKSIM_TRC_AV_SPAWN_ENTERING:-280}" \
    PARKSIM_SUITE_AV_SPAWN_EXITING="${PARKSIM_TRC_AV_SPAWN_EXITING:-200}" \
    PARKSIM_SUITE_RESTORE_OBSTACLES_AS_EXIT_VEHICLES=true \
    PARKSIM_SUITE_HUMAN_INTENT_HIDDEN_FRACTION=0.75 \
    PARKSIM_SUITE_MAX_CONCURRENT_BACKGROUND_VEHICLES=120 \
    PARKSIM_SUITE_REQUIRE_COMPLETE=0 \
    PARKSIM_SUITE_EARLY_STOP=0 \
    PARKSIM_SUITE_TRAFFIC_COMPLETION_STOP=0 \
    PARKSIM_SUITE_TRAFFIC_HORIZON_STOP=1 \
    PARKSIM_SUITE_TRAFFIC_HORIZON_OVERRUN_SECONDS=0 \
    PARKSIM_SUITE_GATE_PROFILE=calibration \
    PARKSIM_SUITE_GATE_STRICT=1 \
    PARKSIM_SUITE_GATE_REQUIRE_VIDEO=0 \
    PARKSIM_SUITE_GATE_SKIP_VIDEO=1 \
    PARKSIM_SUITE_GATE_SKIP_MANUSCRIPT=1 \
    PARKSIM_SUITE_TRC_ANALYSIS=1 \
    PARKSIM_SUITE_CRITICAL_STATE_ANALYSIS=1 \
    PARKSIM_SUITE_WINDOW_METRICS=1 \
    PARKSIM_SUITE_WINDOW_SECONDS=300.0 \
    PARKSIM_SUITE_RESUME=1 \
    PARKSIM_SUITE_CONTINUE_ON_FAIL=0 \
    PARKSIM_SUITE_PROTECT_ACTIVE_TRAINING=1 \
    PARKSIM_SUITE_ALLOW_DURING_TRAINING=0 \
    PARKSIM_SUITE_DRY_RUN="$DRY_RUN" \
    "$ROOT/scripts/run_vla_paper_suite.sh"
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "trc_stage_dry_run=$stage out_dir=$out_dir"
    return 0
  fi
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.staged_suite \
    validate-stage "$out_dir" --stage "$stage" --strict
  echo "trc_stage=$stage out_dir=$out_dir"
}

finalize() {
  parksim_guard_active_training "ParkSim TR-C staged finalization"
  local aggregate="$STAGED_ROOT/aggregate"
  local report="$aggregate/reports/paper_report"
  local stage_dirs=(
    "$STAGED_ROOT/stages/core"
    "$STAGED_ROOT/stages/mixed_density"
    "$STAGED_ROOT/stages/human_behavior"
    "$STAGED_ROOT/stages/replication"
  )
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.staged_suite \
    prepare-aggregate "$aggregate" --inputs "${stage_dirs[@]}" --strict
  mapfile -t benchmark_dirs < "$aggregate/benchmark_dirs.txt"
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.paper_report \
    "$report" --inputs "${benchmark_dirs[@]}" --reference-agent mllm_external_feedback
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.trc_analysis \
    "$report" --out-dir "$report" --reference-agent mllm_external_feedback
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.window_metrics \
    --suite-dir "$aggregate" --out-dir "$report/critical_states" --window-seconds 300.0 \
    --baseline-agent rule_based --target-agent mllm_external_feedback
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.critical_state_analysis \
    --report-dir "$report" --suite-dir "$aggregate" --out-dir "$report/critical_states" \
    --baseline-agent rule_based --target-agent mllm_external_feedback
  local gate_args=("$aggregate" --profile trc --strict)
  if [[ "${PARKSIM_TRC_FINAL_REQUIRE_VIDEO:-1}" != "1" ]]; then
    gate_args+=(--skip-video-evidence)
  fi
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.paper_gate "${gate_args[@]}"
  echo "trc_staged_aggregate=$aggregate"
}

case "$ACTION" in
  core)
    run_stage core "0 1 2 3 4" mixed "medium:450:338:11.4286:15.7143:26.6667:36.6667"
    ;;
  mixed_density)
    run_stage mixed_density "0 1 2 3 4" mixed "low:300:225:17.1429:23.5714:40.0:55.0 high:600:450:8.5714:11.7857:20.0:27.5"
    ;;
  human_behavior)
    run_stage human_behavior "0 1 2 3 4" "rule_random replay" "$DENSITIES"
    ;;
  replication)
    run_stage replication "5 6 7 8 9" "rule_random mixed replay" "$DENSITIES"
    ;;
  finalize)
    finalize
    ;;
  *)
    usage >&2
    exit 64
    ;;
esac
