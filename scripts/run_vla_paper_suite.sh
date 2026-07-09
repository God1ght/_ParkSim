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
QWEN_PORT="${PARKSIM_SUITE_QWEN_PORT:-18100}"
QWEN_BENCH_MODE="$QWEN_MODE"
QWEN_MANAGED="0"
QWEN_READY="0"
qwen_pid=""
CONTROLLED_EGO_BLOCKS_ENTRANCE="${PARKSIM_SUITE_CONTROLLED_EGO_BLOCKS_ENTRANCE:-true}"
REQUIRE_COMPLETE="${PARKSIM_SUITE_REQUIRE_COMPLETE:-0}"
REFERENCE_AGENT="${PARKSIM_SUITE_REFERENCE_AGENT:-qwen_vla}"
REPLAY_ALL_DENSITIES="${PARKSIM_SUITE_REPLAY_ALL_DENSITIES:-0}"
REPORT_NAME="${PARKSIM_SUITE_REPORT_NAME:-paper_report}"
GATE_PROFILE="${PARKSIM_SUITE_GATE_PROFILE:-pilot}"
GATE_STRICT="${PARKSIM_SUITE_GATE_STRICT:-0}"
RESUME="${PARKSIM_SUITE_RESUME:-1}"
DRY_RUN="${PARKSIM_SUITE_DRY_RUN:-0}"
CONTINUE_ON_FAIL="${PARKSIM_SUITE_CONTINUE_ON_FAIL:-0}"
EARLY_STOP="${PARKSIM_SUITE_EARLY_STOP:-1}"
EARLY_STOP_POLL_SECONDS="${PARKSIM_SUITE_EARLY_STOP_POLL_SECONDS:-1}"
EARLY_STOP_GRACE_SECONDS="${PARKSIM_SUITE_EARLY_STOP_GRACE_SECONDS:-2}"

mkdir -p "$OUT_DIR/benchmarks" "$OUT_DIR/reports"
: > "$OUT_DIR/benchmarks.jsonl"
: > "$OUT_DIR/failures.jsonl"
if [[ "$RESUME" != "1" ]]; then
  rm -f "$OUT_DIR/qwen_health.json" "$OUT_DIR/qwen_service.log"
fi

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
    "qwen_port": os.environ.get("QWEN_PORT", ""),
    "qwen_bench_mode": os.environ.get("QWEN_BENCH_MODE", ""),
    "qwen_managed": os.environ.get("QWEN_MANAGED", ""),
    "qwen_ready": os.environ.get("QWEN_READY", ""),
    "qwen_health_path": os.path.join(os.path.dirname(sys.argv[1]), "qwen_health.json"),
    "qwen_service_log": os.path.join(os.path.dirname(sys.argv[1]), "qwen_service.log"),
    "controlled_ego_blocks_entrance": os.environ.get("CONTROLLED_EGO_BLOCKS_ENTRANCE", ""),
    "require_complete": os.environ.get("REQUIRE_COMPLETE", ""),
    "reference_agent": os.environ.get("REFERENCE_AGENT", ""),
    "replay_all_densities": os.environ.get("REPLAY_ALL_DENSITIES", ""),
    "gate_profile": os.environ.get("GATE_PROFILE", ""),
    "gate_strict": os.environ.get("GATE_STRICT", ""),
    "resume": os.environ.get("RESUME", ""),
    "dry_run": os.environ.get("DRY_RUN", ""),
    "continue_on_fail": os.environ.get("CONTINUE_ON_FAIL", ""),
    "early_stop": os.environ.get("EARLY_STOP", ""),
    "early_stop_poll_seconds": os.environ.get("EARLY_STOP_POLL_SECONDS", ""),
    "early_stop_grace_seconds": os.environ.get("EARLY_STOP_GRACE_SECONDS", ""),
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

needs_qwen() {
  for agent in $AGENTS; do
    if [[ "$agent" == "qwen_vla" ]]; then
      return 0
    fi
  done
  return 1
}

cleanup() {
  if [[ -n "$qwen_pid" ]] && kill -0 "$qwen_pid" 2>/dev/null; then
    kill "$qwen_pid" 2>/dev/null || true
    wait "$qwen_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

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
      echo "Suite Qwen service exited early" >&2
      tail -n 120 "$OUT_DIR/qwen_service.log" >&2 || true
      exit 1
    fi
    sleep 1
  done
  if [[ "$ready" != "1" ]]; then
    echo "Suite Qwen service did not become healthy: $endpoint" >&2
    tail -n 120 "$OUT_DIR/qwen_service.log" >&2 || true
    exit 1
  fi
}

start_qwen_if_needed() {
  if ! needs_qwen || [[ "$DRY_RUN" == "1" || "$QWEN_READY" == "1" ]]; then
    return
  fi
  if [[ -n "$QWEN_ENDPOINT" ]]; then
    wait_for_qwen "$QWEN_ENDPOINT" "any"
    QWEN_BENCH_MODE="external"
    QWEN_READY="1"
    export QWEN_ENDPOINT QWEN_BENCH_MODE QWEN_MANAGED QWEN_READY
    write_suite_config
    return
  fi
  case "$QWEN_MODE" in
    real)
      QWEN_ENDPOINT="http://127.0.0.1:$QWEN_PORT/v1/chat/completions"
      QWEN_VLA_PORT="$QWEN_PORT" "$ROOT/scripts/run_qwen_vla_service.sh" > "$OUT_DIR/qwen_service.log" 2>&1 &
      qwen_pid="$!"
      QWEN_MANAGED="1"
      wait_for_qwen "$QWEN_ENDPOINT" "false"
      QWEN_BENCH_MODE="external"
      QWEN_READY="1"
      ;;
    mock)
      QWEN_BENCH_MODE="mock"
      QWEN_READY="1"
      ;;
    external)
      echo "PARKSIM_SUITE_QWEN_MODE=external requires PARKSIM_SUITE_QWEN_ENDPOINT" >&2
      exit 1
      ;;
    *)
      echo "Unsupported PARKSIM_SUITE_QWEN_MODE: $QWEN_MODE" >&2
      exit 1
      ;;
  esac
  export QWEN_ENDPOINT QWEN_BENCH_MODE QWEN_MANAGED QWEN_READY
  write_suite_config
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
qwen_health_path = out_dir / "qwen_health.json"
qwen_service_log = out_dir / "qwen_service.log"
qwen_health = None
qwen_health_error = ""
if qwen_health_path.exists():
    try:
        qwen_health_text = qwen_health_path.read_text().strip()
        qwen_health = json.loads(qwen_health_text) if qwen_health_text else None
        if not qwen_health_text:
            qwen_health_error = "empty qwen_health.json"
    except Exception as exc:
        qwen_health_error = str(exc)
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
    "paper_gate": {
        "dir": str(out_dir / "reports" / "paper_gate"),
        "json": str(out_dir / "reports" / "paper_gate" / "paper_gate.json"),
        "markdown": str(out_dir / "reports" / "paper_gate" / "paper_gate.md"),
        "exists": (out_dir / "reports" / "paper_gate" / "paper_gate.json").exists(),
    },
    "qwen": {
        "mode": os.environ.get("QWEN_MODE", ""),
        "bench_mode": os.environ.get("QWEN_BENCH_MODE", ""),
        "managed": os.environ.get("QWEN_MANAGED", ""),
        "ready": os.environ.get("QWEN_READY", ""),
        "endpoint": os.environ.get("QWEN_ENDPOINT", ""),
        "health_path": str(qwen_health_path),
        "health": qwen_health,
        "health_error": qwen_health_error,
        "service_log": str(qwen_service_log),
        "service_log_exists": qwen_service_log.exists(),
    },
}
(out_dir / "suite_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print("suite_manifest=%s benchmarks=%d all_valid=%s" % (out_dir / "suite_manifest.json", len(benchmarks), manifest["all_valid"]))
SUITE_MANIFEST_JSON
}

export ROOT GIT_COMMIT GIT_BRANCH AGENTS SEEDS BACKGROUND_MODES DENSITY_CONFIGS DURATION SPOT_INDEX QWEN_MODE QWEN_ENDPOINT QWEN_TIMEOUT QWEN_STARTUP_TIMEOUT QWEN_PORT QWEN_BENCH_MODE QWEN_MANAGED QWEN_READY CONTROLLED_EGO_BLOCKS_ENTRANCE REQUIRE_COMPLETE REFERENCE_AGENT REPLAY_ALL_DENSITIES REPORT_NAME GATE_PROFILE GATE_STRICT RESUME DRY_RUN CONTINUE_ON_FAIL EARLY_STOP EARLY_STOP_POLL_SECONDS EARLY_STOP_GRACE_SECONDS
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
    start_qwen_if_needed
    set +e
    PARKSIM_BENCH_OUT_DIR="$benchmark_dir" \
    PARKSIM_BENCH_AGENTS="$AGENTS" \
    PARKSIM_BENCH_SEEDS="$SEEDS" \
    PARKSIM_BENCH_BACKGROUND_MODES="$background_mode" \
    PARKSIM_BENCH_DURATION="$DURATION" \
    PARKSIM_BENCH_SPOT_INDEX="$SPOT_INDEX" \
    PARKSIM_BENCH_SPAWN_ENTERING="$spawn_entering" \
    PARKSIM_BENCH_SPAWN_EXITING="$spawn_exiting" \
    PARKSIM_BENCH_QWEN_MODE="$QWEN_BENCH_MODE" \
    PARKSIM_BENCH_QWEN_ENDPOINT="$QWEN_ENDPOINT" \
    PARKSIM_BENCH_QWEN_TIMEOUT="$QWEN_TIMEOUT" \
    PARKSIM_BENCH_QWEN_STARTUP_TIMEOUT="$QWEN_STARTUP_TIMEOUT" \
    PARKSIM_BENCH_CONTROLLED_EGO_BLOCKS_ENTRANCE="$CONTROLLED_EGO_BLOCKS_ENTRANCE" \
    PARKSIM_BENCH_REQUIRE_COMPLETE="$REQUIRE_COMPLETE" \
    PARKSIM_BENCH_EARLY_STOP="$EARLY_STOP" \
    PARKSIM_BENCH_EARLY_STOP_POLL_SECONDS="$EARLY_STOP_POLL_SECONDS" \
    PARKSIM_BENCH_EARLY_STOP_GRACE_SECONDS="$EARLY_STOP_GRACE_SECONDS" \
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
write_suite_config
write_suite_manifest

gate_args=("$OUT_DIR" --profile "$GATE_PROFILE")
if [[ "$GATE_STRICT" == "1" ]]; then
  gate_args+=(--strict)
fi
PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" python3 -m parksim.vla.paper_gate "${gate_args[@]}"
write_suite_manifest

echo "paper_suite_out_dir=$OUT_DIR"
find "$OUT_DIR" -maxdepth 3 \( -name 'suite_manifest.json' -o -name 'suite_config.json' -o -name 'paper_summary.md' -o -name 'paper_table.tex' -o -name 'paper_report_manifest.json' -o -name 'paper_statistical_tests.csv' -o -name 'paper_stratified_summary.csv' -o -name 'paper_reproducibility.json' -o -name 'paper_gate.json' -o -name 'paper_gate.md' -o -name 'qwen_health.json' -o -name 'qwen_service.log' \) -print | sort
