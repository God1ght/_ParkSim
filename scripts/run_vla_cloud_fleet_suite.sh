#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="${PARKSIM_CLOUD_SUITE_OUT_DIR:-$ROOT/experiments/qwen_vla_cloud_fleet_suite/$(date +%Y%m%d_%H%M%S)}"
AGENTS="${PARKSIM_CLOUD_SUITE_AGENTS:-rule_based centralized_min_cost qwen_vla}"
SEEDS="${PARKSIM_CLOUD_SUITE_SEEDS:-0 1 2}"
BACKGROUND_MODES="${PARKSIM_CLOUD_SUITE_BACKGROUND_MODES:-rule_random mixed}"
DENSITIES="${PARKSIM_CLOUD_SUITE_DENSITIES:-light:6:6:2:2 balanced:16:16:6:6 dense:32:32:10:10}"
DURATION="${PARKSIM_CLOUD_SUITE_DURATION:-300s}"
WALL_TIMEOUT="${PARKSIM_CLOUD_SUITE_WALL_TIMEOUT:-1800s}"
QWEN_ENDPOINT="${PARKSIM_CLOUD_SUITE_QWEN_ENDPOINT:-}"
QWEN_TIMEOUT="${PARKSIM_CLOUD_SUITE_QWEN_TIMEOUT:-120.0}"
REQUIRE_COMPLETE="${PARKSIM_CLOUD_SUITE_REQUIRE_COMPLETE:-1}"
REPORT_NAME="${PARKSIM_CLOUD_SUITE_REPORT_NAME:-cloud_fleet_report}"
CONTINUE_ON_FAIL="${PARKSIM_CLOUD_SUITE_CONTINUE_ON_FAIL:-1}"

AGENTS="${AGENTS//,/ }"
mkdir -p "$OUT_DIR/benchmarks"
: > "$OUT_DIR/benchmarks.jsonl"
: > "$OUT_DIR/failures.jsonl"

if [[ "$AGENTS" == *qwen_vla* && -z "$QWEN_ENDPOINT" ]]; then
  echo "PARKSIM_CLOUD_SUITE_QWEN_ENDPOINT is required for real cloud-fleet Qwen experiments." >&2
  exit 1
fi

git_commit="$(git -C "$ROOT" rev-parse HEAD)"
export git_commit AGENTS SEEDS BACKGROUND_MODES DENSITIES DURATION WALL_TIMEOUT QWEN_ENDPOINT QWEN_TIMEOUT REQUIRE_COMPLETE
python3 - "$OUT_DIR/suite_config.json" <<'PYCONFIG'
import json
import os
import sys

payload = {
    "type": "parksim_qwen_vla_cloud_fleet_suite",
    "git_commit": os.environ["git_commit"],
    "agents": os.environ["AGENTS"].split(),
    "seeds": os.environ["SEEDS"].split(),
    "background_modes": os.environ["BACKGROUND_MODES"].split(),
    "densities": os.environ["DENSITIES"].split(),
    "duration": os.environ["DURATION"],
    "wall_timeout": os.environ["WALL_TIMEOUT"],
    "qwen_endpoint": os.environ["QWEN_ENDPOINT"],
    "qwen_timeout": os.environ["QWEN_TIMEOUT"],
    "require_complete": os.environ["REQUIRE_COMPLETE"],
}
with open(sys.argv[1], "w") as handle:
    json.dump(payload, handle, indent=2)
PYCONFIG

append_record() {
  local benchmark_dir="$1"
  local background_mode="$2"
  local density_label="$3"
  local human_entering="$4"
  local human_exiting="$5"
  local av_entering="$6"
  local av_exiting="$7"
  local status="$8"
  local detail="${9:-}"
  python3 - "$OUT_DIR/benchmarks.jsonl" "$benchmark_dir" "$background_mode" "$density_label" "$human_entering" "$human_exiting" "$av_entering" "$av_exiting" "$status" "$detail" <<'PYRECORD'
import json
import sys

record = {
    "benchmark_dir": sys.argv[2],
    "background_mode": sys.argv[3],
    "density_label": sys.argv[4],
    "human_entering": int(sys.argv[5]),
    "human_exiting": int(sys.argv[6]),
    "av_entering": int(sys.argv[7]),
    "av_exiting": int(sys.argv[8]),
    "status": sys.argv[9],
    "detail": sys.argv[10],
}
with open(sys.argv[1], "a") as handle:
    handle.write(json.dumps(record) + "\n")
PYRECORD
}

benchmark_dirs=()
for background_mode in $BACKGROUND_MODES; do
  for density in $DENSITIES; do
    IFS=":" read -r density_label human_entering human_exiting av_entering av_exiting <<< "$density"
    if [[ -z "$density_label" || -z "$human_entering" || -z "$human_exiting" || -z "$av_entering" || -z "$av_exiting" ]]; then
      echo "Invalid density '$density'. Use label:human_enter:human_exit:av_enter:av_exit" >&2
      exit 1
    fi
    benchmark_dir="$OUT_DIR/benchmarks/${background_mode}_${density_label}_h${human_entering}_${human_exiting}_av${av_entering}_${av_exiting}"
    set +e
    PARKSIM_BENCH_OUT_DIR="$benchmark_dir" \
    PARKSIM_BENCH_CLEAR_OUT_DIR=1 \
    PARKSIM_BENCH_AGENTS="$AGENTS" \
    PARKSIM_BENCH_SEEDS="$SEEDS" \
    PARKSIM_BENCH_BACKGROUND_MODES="$background_mode" \
    PARKSIM_BENCH_DURATION="$DURATION" \
    PARKSIM_BENCH_WALL_TIMEOUT="$WALL_TIMEOUT" \
    PARKSIM_BENCH_TRAFFIC_FLOW_MODE=human_mixed_long_horizon \
    PARKSIM_BENCH_SPAWN_ENTERING="$human_entering" \
    PARKSIM_BENCH_SPAWN_EXITING="$human_exiting" \
    PARKSIM_BENCH_AV_SPAWN_ENTERING="$av_entering" \
    PARKSIM_BENCH_AV_SPAWN_EXITING="$av_exiting" \
    PARKSIM_BENCH_ENTRY_VEHICLE_AGENT_TYPE=rule_based \
    PARKSIM_BENCH_EXIT_VEHICLE_AGENT_TYPE=rule_based \
    PARKSIM_BENCH_AV_ENTRY_VEHICLE_AGENT_TYPE=__agent__ \
    PARKSIM_BENCH_AV_EXIT_VEHICLE_AGENT_TYPE=__agent__ \
    PARKSIM_BENCH_SPAWN_CONTROLLED_EGO=false \
    PARKSIM_BENCH_LONG_HORIZON_DURATION="${DURATION%s}" \
    PARKSIM_BENCH_RESTORE_OBSTACLES_AS_EXIT_VEHICLES=true \
    PARKSIM_BENCH_HUMAN_INTENT_HIDDEN_FRACTION=0.75 \
    PARKSIM_BENCH_QWEN_ENDPOINT="$QWEN_ENDPOINT" \
    PARKSIM_BENCH_QWEN_TIMEOUT="$QWEN_TIMEOUT" \
    PARKSIM_BENCH_REQUIRE_COMPLETE="$REQUIRE_COMPLETE" \
    PARKSIM_BENCH_EARLY_STOP=0 \
    PARKSIM_BENCH_TRAFFIC_COMPLETION_STOP=1 \
    PARKSIM_BENCH_TRAFFIC_HORIZON_STOP=1 \
      "$ROOT/scripts/run_vla_benchmark.sh"
    status="$?"
    set -e
    if [[ "$status" == "0" ]]; then
      benchmark_dirs+=("$benchmark_dir")
      append_record "$benchmark_dir" "$background_mode" "$density_label" "$human_entering" "$human_exiting" "$av_entering" "$av_exiting" completed
    else
      append_record "$benchmark_dir" "$background_mode" "$density_label" "$human_entering" "$human_exiting" "$av_entering" "$av_exiting" failed "exit_status=$status"
      if [[ "$CONTINUE_ON_FAIL" != "1" ]]; then
        exit "$status"
      fi
    fi
  done
done

if (( ${#benchmark_dirs[@]} == 0 )); then
  echo "No valid cloud-fleet benchmarks completed." >&2
  exit 1
fi

report_dir="$OUT_DIR/reports/$REPORT_NAME"
PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.paper_report "$report_dir" --inputs "${benchmark_dirs[@]}" --reference-agent qwen_vla
PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.trc_analysis "$report_dir" --out-dir "$report_dir" --reference-agent qwen_vla
echo "cloud_fleet_suite_out_dir=$OUT_DIR"
