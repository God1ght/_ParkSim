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

mkdir -p "$OUT_DIR/benchmarks" "$OUT_DIR/reports"
: > "$OUT_DIR/benchmarks.jsonl"

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
  python3 - "$OUT_DIR/benchmarks.jsonl" "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting" <<'BENCHMARK_JSON'
import json
import sys
path, benchmark_dir, background_mode, density_label, spawn_entering, spawn_exiting = sys.argv[1:]
record = {
    "benchmark_dir": benchmark_dir,
    "background_mode": background_mode,
    "density_label": density_label,
    "spawn_entering": int(spawn_entering),
    "spawn_exiting": int(spawn_exiting),
}
with open(path, "a") as f:
    f.write(json.dumps(record) + "\n")
BENCHMARK_JSON
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

export ROOT GIT_COMMIT GIT_BRANCH AGENTS SEEDS BACKGROUND_MODES DENSITY_CONFIGS DURATION SPOT_INDEX QWEN_MODE QWEN_ENDPOINT QWEN_TIMEOUT CONTROLLED_EGO_BLOCKS_ENTRANCE REQUIRE_COMPLETE REFERENCE_AGENT REPLAY_ALL_DENSITIES REPORT_NAME
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
    benchmark_dirs+=("$benchmark_dir")
    append_benchmark_manifest "$benchmark_dir" "$background_mode" "$density_label" "$spawn_entering" "$spawn_exiting"
  done
done

if (( ${#benchmark_dirs[@]} == 0 )); then
  echo "No benchmarks were run. Check BACKGROUND_MODES, DENSITY_CONFIGS, and replay density skip settings." >&2
  exit 1
fi

report_dir="$OUT_DIR/reports/$REPORT_NAME"
PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.paper_report "$report_dir" --inputs "${benchmark_dirs[@]}" --reference-agent "$REFERENCE_AGENT"
write_suite_manifest

echo "paper_suite_out_dir=$OUT_DIR"
find "$OUT_DIR" -maxdepth 3 \( -name 'suite_manifest.json' -o -name 'suite_config.json' -o -name 'paper_summary.md' -o -name 'paper_table.tex' -o -name 'paper_report_manifest.json' \) -print | sort
