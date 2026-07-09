#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT_DIR="${PARKSIM_SUITE_OUT_DIR:-$ROOT/experiments/qwen_vla_paper_suite/$(date +%Y%m%d_%H%M%S)}"
AGENTS="${PARKSIM_SUITE_AGENTS:-rule_based greedy_nearest greedy_shortest_path risk_aware_rule qwen_vla}"
SEEDS="${PARKSIM_SUITE_SEEDS:-0 1 2}"
BACKGROUND_MODES="${PARKSIM_SUITE_BACKGROUND_MODES:-rule_random mixed replay}"
DENSITY_CONFIGS="${PARKSIM_SUITE_DENSITY_CONFIGS:-empty:0:0 light:1:0 balanced:2:2 dense:4:4}"
DURATION="${PARKSIM_SUITE_DURATION:-120s}"
SPOT_INDEX="${PARKSIM_SUITE_SPOT_INDEX:-7}"
QWEN_MODE="${PARKSIM_SUITE_QWEN_MODE:-mock}"
QWEN_ENDPOINT="${PARKSIM_SUITE_QWEN_ENDPOINT:-}"
QWEN_TIMEOUT="${PARKSIM_SUITE_QWEN_TIMEOUT:-120.0}"
QWEN_STARTUP_TIMEOUT="${PARKSIM_SUITE_QWEN_STARTUP_TIMEOUT:-900}"
CONTROLLED_EGO_BLOCKS_ENTRANCE="${PARKSIM_SUITE_CONTROLLED_EGO_BLOCKS_ENTRANCE:-true}"
REQUIRE_COMPLETE="${PARKSIM_SUITE_REQUIRE_COMPLETE:-0}"
REFERENCE_AGENT="${PARKSIM_SUITE_REFERENCE_AGENT:-qwen_vla}"
REPLAY_ALL_DENSITIES="${PARKSIM_SUITE_REPLAY_ALL_DENSITIES:-0}"
REPORT_NAME="${PARKSIM_SUITE_REPORT_NAME:-paper_report}"
RESUME="${PARKSIM_SUITE_RESUME:-1}"
DRY_RUN="${PARKSIM_SUITE_DRY_RUN:-0}"
CONTINUE_ON_FAIL="${PARKSIM_SUITE_CONTINUE_ON_FAIL:-0}"

mkdir -p "$OUT_DIR/benchmarks" "$OUT_DIR/reports"
: > "$OUT_DIR/benchmarks.jsonl"
: > "$OUT_DIR/failures.jsonl"

GIT_COMMIT="$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo unknown)"
GIT_BRANCH="$(git -C "$ROOT" branch --show-current 2>/dev/null || echo unknown)"

write_suite_config() {
  python3 - "$OUT_DIR/suite_config.json" <<'SUITE_CONFIG_JSON'
import json
import os
import sys
payload = {
    "root": os.environ.get("ROOT", ""),
    "git_commit": os.environ.get("GIT_COMMIT", ""),
    "git_branch": os.environ.get("GIT_BRANCH", ""),
    "agents": os.environ.get("AGENTS", "").split(),
    "seeds": os.environ.get("SEEDS", "").split(),
    "background_modes": os.environ.get("BACKGROUND_MODES", "").split(),
    "density_configs": os.environ.get("DENSITY_CONFIGS", "").split(),
    "duration": os.environ.get("DURATION", ""),
    "spot_index": os.environ.get("SPOT_INDEX", ""),
    "qwen_mode": os.environ.get("QWEN_MODE", ""),
    "qwen_endpoint": os.environ.get("QWEN_ENDPOINT", ""),
    "qwen_timeout": os.environ.get("QWEN_TIMEOUT", ""),
    "controlled_ego_blocks_entrance": os.environ.get("CONTROLLED_EGO_BLOCKS_ENTRANCE", ""),
    "require_complete": os.environ.get("REQUIRE_COMPLETE", ""),
    "reference_agent": os.environ.get("REFERENCE_AGENT", ""),
    "replay_all_densities": os.environ.get("REPLAY_ALL_DENSITIES", ""),
    "resume": os.environ.get("RESUME", ""),
    "dry_run": os.environ.get("DRY_RUN", ""),
    "continue_on_fail": os.environ.get("CONTINUE_ON_FAIL", ""),
}
with open(sys.argv[1], "w") as f:
    json.dump(payload, f, indent=2)
SUITE_CONFIG_JSON
}

append_benchmark_manifest() {
  local benchmark_dir="$1"
  local background_mode="$2"
  local density_label="$3"
  local spawn_entering="$4"
  local spawn_exiting="$5"
  local status="$6"
  local reason="${7:-}"
  python3 - "$OUT_DIR/benchmarks.jsonl" "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting" "$status" "$reason" <<'BENCHMARK_JSON'
import json
import sys
path, benchmark_dir, background_mode, density_label, spawn_entering, spawn_exiting, status, reason = sys.argv[1:]
record = {
    "benchmark_dir": benchmark_dir,
    "background_mode": background_mode,
    "density_label": density_label,
    "spawn_entering": int(spawn_entering),
    "spawn_exiting": int(spawn_exiting),
    "status": status,
}
if reason:
    record["reason"] = reason
with open(path, "a") as f:
    f.write(json.dumps(record) + "\n")
BENCHMARK_JSON
}

append_failure() {
  local benchmark_dir="$1"
  local background_mode="$2"
  local density_label="$3"
  local spawn_entering="$4"
  local spawn_exiting="$5"
  local exit_status="$6"
  python3 - "$OUT_DIR/failures.jsonl" "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting" "$exit_status" <<'FAILURE_JSON'
import json
import sys
path, benchmark_dir, background_mode, density_label, spawn_entering, spawn_exiting, exit_status = sys.argv[1:]
record = {
    "benchmark_dir": benchmark_dir,
    "background_mode": background_mode,
    "density_label": density_label,
    "spawn_entering": int(spawn_entering),
    "spawn_exiting": int(spawn_exiting),
    "exit_status": int(exit_status),
}
with open(path, "a") as f:
    f.write(json.dumps(record) + "\n")
FAILURE_JSON
}

validate_benchmark_dir() {
  local benchmark_dir="$1"
  local expected_agents_csv
  expected_agents_csv="$(printf '%s' "$AGENTS" | tr ' ' ',')"
  local validate_args=("$benchmark_dir" --expected-agents "$expected_agents_csv")
  if [[ "$REQUIRE_COMPLETE" == "1" ]]; then
    validate_args+=(--require-complete)
  fi
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.benchmark validate "${validate_args[@]}" >/dev/null 2>&1
}

write_suite_manifest() {
  python3 - "$OUT_DIR" <<'SUITE_MANIFEST_JSON'
import json
import sys
from pathlib import Path
out_dir = Path(sys.argv[1])
benchmarks = []
benchmarks_path = out_dir / "benchmarks.jsonl"
if benchmarks_path.exists():
    for line in benchmarks_path.read_text().splitlines():
        if line.strip():
            item = json.loads(line)
            benchmark_dir = Path(item["benchmark_dir"])
            validation_path = benchmark_dir / "validation.json"
            item["validation_ok"] = False
            item["validation_path"] = str(validation_path)
            if validation_path.exists():
                item["validation"] = json.loads(validation_path.read_text())
                item["validation_ok"] = bool(item["validation"].get("ok"))
            benchmarks.append(item)
import os
report_dir = out_dir / "reports" / os.environ.get("REPORT_NAME", "paper_report")
manifest = {
    "type": "parksim_vla_paper_suite",
    "out_dir": str(out_dir),
    "suite_config": str(out_dir / "suite_config.json"),
    "benchmark_count": len(benchmarks),
    "status_counts": {status: sum(1 for item in benchmarks if item.get("status") == status) for status in sorted({item.get("status") for item in benchmarks})},
    "benchmarks": benchmarks,
    "all_valid": all(item.get("validation_ok") for item in benchmarks) if benchmarks else False,
    "paper_report": {
        "dir": str(report_dir),
        "summary_md": str(report_dir / "paper_summary.md"),
        "table_tex": str(report_dir / "paper_table.tex"),
        "manifest": str(report_dir / "paper_report_manifest.json"),
    },
}
(out_dir / "suite_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print("suite_manifest=%s benchmarks=%d all_valid=%s" % (out_dir / "suite_manifest.json", len(benchmarks), manifest["all_valid"]))
SUITE_MANIFEST_JSON
}

export ROOT GIT_COMMIT GIT_BRANCH AGENTS SEEDS BACKGROUND_MODES DENSITY_CONFIGS DURATION SPOT_INDEX QWEN_MODE QWEN_ENDPOINT QWEN_TIMEOUT CONTROLLED_EGO_BLOCKS_ENTRANCE REQUIRE_COMPLETE REFERENCE_AGENT REPLAY_ALL_DENSITIES REPORT_NAME RESUME DRY_RUN CONTINUE_ON_FAIL
write_suite_config

benchmark_dirs=()
for background_mode in $BACKGROUND_MODES; do
  for density in $DENSITY_CONFIGS; do
    IFS=":" read -r density_label spawn_entering spawn_exiting <<< "$density"
    if [[ -z "$density_label" || -z "$spawn_entering" || -z "$spawn_exiting" ]]; then
      echo "Invalid density config '$density'. Use label:spawn_entering:spawn_exiting" >&2
      exit 1
    fi
    if [[ "$background_mode" == "replay" && "$density_label" != "empty" && "$REPLAY_ALL_DENSITIES" != "1" ]]; then
      echo "skip replay density=$density_label because replay ignores random spawn density"
      continue
    fi
    benchmark_dir="$OUT_DIR/benchmarks/${background_mode}_${density_label}_enter${spawn_entering}_exit${spawn_exiting}"
    echo "suite benchmark background=$background_mode density=$density_label enter=$spawn_entering exit=$spawn_exiting -> $benchmark_dir"
    if [[ "$DRY_RUN" == "1" ]]; then
      append_benchmark_manifest "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting" planned "dry-run"
      continue
    fi
    if [[ "$RESUME" == "1" && -d "$benchmark_dir" ]] && validate_benchmark_dir "$benchmark_dir"; then
      echo "skip valid benchmark -> $benchmark_dir"
      benchmark_dirs+=("$benchmark_dir")
      append_benchmark_manifest "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting" skipped_valid "resume"
      continue
    fi
    set +e
    PARKSIM_BENCH_OUT_DIR="$benchmark_dir" \
    PARKSIM_BENCH_AGENTS="$AGENTS" \
    PARKSIM_BENCH_SEEDS="$SEEDS" \
    PARKSIM_BENCH_BACKGROUND_MODES="$background_mode" \
    PARKSIM_BENCH_DURATION="$DURATION" \
    PARKSIM_BENCH_SPOT_INDEX="$SPOT_INDEX" \
    PARKSIM_BENCH_SPAWN_ENTERING="$spawn_entering" \
    PARKSIM_BENCH_SPAWN_EXITING="$spawn_exiting" \
    PARKSIM_BENCH_QWEN_MODE="$QWEN_MODE" \
    PARKSIM_BENCH_QWEN_ENDPOINT="$QWEN_ENDPOINT" \
    PARKSIM_BENCH_QWEN_TIMEOUT="$QWEN_TIMEOUT" \
    PARKSIM_BENCH_QWEN_STARTUP_TIMEOUT="$QWEN_STARTUP_TIMEOUT" \
    PARKSIM_BENCH_CONTROLLED_EGO_BLOCKS_ENTRANCE="$CONTROLLED_EGO_BLOCKS_ENTRANCE" \
    PARKSIM_BENCH_REQUIRE_COMPLETE="$REQUIRE_COMPLETE" \
      "$ROOT/scripts/run_vla_benchmark.sh"
    status="$?"
    set -e
    if [[ "$status" != "0" ]]; then
      append_failure "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting" "$status"
      append_benchmark_manifest "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting" failed "exit_status=$status"
      if [[ "$CONTINUE_ON_FAIL" != "1" ]]; then
        exit "$status"
      fi
      continue
    fi
    benchmark_dirs+=("$benchmark_dir")
    append_benchmark_manifest "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting" completed
  done
done

if [[ "$DRY_RUN" == "1" ]]; then
  write_suite_manifest
  echo "paper_suite_dry_run_out_dir=$OUT_DIR"
  exit 0
fi

if (( ${#benchmark_dirs[@]} == 0 )); then
  write_suite_manifest
  echo "No valid benchmarks are available for reporting. Check BACKGROUND_MODES, DENSITY_CONFIGS, resume status, and failures.jsonl." >&2
  exit 1
fi

report_dir="$OUT_DIR/reports/$REPORT_NAME"
PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.paper_report "$report_dir" --inputs "${benchmark_dirs[@]}" --reference-agent "$REFERENCE_AGENT"
write_suite_manifest

echo "paper_suite_out_dir=$OUT_DIR"
find "$OUT_DIR" -maxdepth 3 \( -name 'suite_manifest.json' -o -name 'suite_config.json' -o -name 'paper_summary.md' -o -name 'paper_table.tex' -o -name 'paper_report_manifest.json' \) -print | sort
