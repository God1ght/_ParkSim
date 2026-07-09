#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ "${PARKSIM_INSIDE_XVFB:-0}" != "1" ]]; then
  exec xvfb-run -a -s "-screen 0 ${PARKSIM_VIS_SCREEN:-2000x1100x24} +extension GLX +render -noreset" \
    env PARKSIM_INSIDE_XVFB=1 LIBGL_ALWAYS_SOFTWARE=1 "$0" "$@"
fi

ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
WORKSPACE_SETUP="${WORKSPACE_SETUP:-$ROOT/workspace/install/setup.bash}"
DLP_ROOT="${PARKSIM_DLP_ROOT:-/media/step/data/Parking_Yccc7/_dlp_dataset}"
DLP_PATH="${PARKSIM_VIS_DLP_PATH:-$ROOT/python/parksim/priorFiles/data/DJI_0012}"
OUT_DIR="${PARKSIM_VIS_OUT_DIR:-$ROOT/experiments/qwen_vla_visualizer_videos/$(date +%Y%m%d_%H%M%S)}"
DURATION="${PARKSIM_VIS_DURATION:-70s}"
SPOT_INDEX="${PARKSIM_VIS_SPOT_INDEX:-7}"
SPAWN_TIME="${PARKSIM_VIS_SPAWN_TIME:-0.5}"
BACKGROUND_MODE="${PARKSIM_VIS_BACKGROUND_MODE:-rule_random}"
SPAWN_ENTERING="${PARKSIM_VIS_SPAWN_ENTERING:-0}"
SPAWN_EXITING="${PARKSIM_VIS_SPAWN_EXITING:-0}"
CONTROLLED_EGO_BLOCKS_ENTRANCE="${PARKSIM_VIS_CONTROLLED_EGO_BLOCKS_ENTRANCE:-true}"
RANDOM_SEED="${PARKSIM_VIS_RANDOM_SEED:-0}"
QWEN_MODE="${PARKSIM_VIS_QWEN_MODE:-mock}"
QWEN_PORT="${PARKSIM_VIS_QWEN_PORT:-18088}"
QWEN_ENDPOINT="${PARKSIM_VIS_QWEN_ENDPOINT:-}"
QWEN_TIMEOUT="${PARKSIM_VIS_QWEN_TIMEOUT:-120.0}"
QWEN_STARTUP_TIMEOUT="${PARKSIM_VIS_QWEN_STARTUP_TIMEOUT:-900}"
VIS_TIMER_PERIOD="${PARKSIM_VIS_TIMER_PERIOD:-0.05}"
RECORD_EVERY_N="${PARKSIM_VIS_RECORD_EVERY_N:-2}"
VIDEO_FPS="${PARKSIM_VIS_VIDEO_FPS:-10}"
GIF_FPS="${PARKSIM_VIS_GIF_FPS:-8}"
ALIGN_DT="${PARKSIM_VIS_ALIGN_DT:-0.1}"
ALIGN_PANEL_WIDTH="${PARKSIM_VIS_ALIGN_PANEL_WIDTH:-960}"
VIS_START_DELAY="${PARKSIM_VIS_START_DELAY:-2}"
VIS_POST_ROLL="${PARKSIM_VIS_POST_ROLL:-2}"
EARLY_STOP="${PARKSIM_VIS_EARLY_STOP:-1}"
EARLY_STOP_POLL_SECONDS="${PARKSIM_VIS_EARLY_STOP_POLL_SECONDS:-1}"
PYTHON_BIN="${PARKSIM_VIS_PYTHON:-/usr/bin/python3}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup file not found: $ROS_SETUP" >&2
  exit 1
fi
if [[ ! -f "$WORKSPACE_SETUP" ]]; then
  echo "Workspace setup file not found: $WORKSPACE_SETUP. Run ./scripts/build_parksim_ros.sh first." >&2
  exit 1
fi
if [[ ! -d "$DLP_ROOT/dlp" ]]; then
  echo "DLP package not found under $DLP_ROOT. Set PARKSIM_DLP_ROOT or copy DLP data to this host." >&2
  exit 1
fi
if [[ ! -f "${DLP_PATH}_scene.json" ]]; then
  echo "DLP scene prefix not found: $DLP_PATH (expected ${DLP_PATH}_scene.json)" >&2
  exit 1
fi
command -v ffmpeg >/dev/null || { echo "ffmpeg is required" >&2; exit 1; }
$PYTHON_BIN -c 'import dearpygui.dearpygui' >/dev/null 2>&1 || { echo "dearpygui is required for visualizer recording" >&2; exit 1; }

set +u
if [[ -n "${CONDA_PREFIX:-}" && -f /home/step/anaconda3/etc/profile.d/conda.sh ]]; then
  source /home/step/anaconda3/etc/profile.d/conda.sh
  conda deactivate >/dev/null 2>&1 || true
fi
source "$ROS_SETUP"
source "$WORKSPACE_SETUP"
hash -r
set -u

mkdir -p "$OUT_DIR"
qwen_pid=""
visualizer_pid=""
cleanup_simulation_processes() {
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/simulator_node.py" 2>/dev/null || true
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/vehicle_node.py" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/simulator_node.py" 2>/dev/null || true
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/vehicle_node.py" 2>/dev/null || true
}

cleanup_ros_processes() {
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/simulator_node.py" 2>/dev/null || true
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/vehicle_node.py" 2>/dev/null || true
  pkill -TERM -f "$ROOT/workspace/install/parksim/lib/parksim/visualizer_node.py" 2>/dev/null || true
  sleep 1
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/simulator_node.py" 2>/dev/null || true
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/vehicle_node.py" 2>/dev/null || true
  pkill -KILL -f "$ROOT/workspace/install/parksim/lib/parksim/visualizer_node.py" 2>/dev/null || true
}
cleanup() {
  cleanup_ros_processes
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
    if $PYTHON_BIN -c 'import json,sys; from urllib import request; url=sys.argv[1]+"/healthz"; expected=sys.argv[2]; resp=request.urlopen(url, timeout=1.0); payload=json.loads(resp.read().decode("utf-8")); print(payload); raise SystemExit(0 if payload.get("ok") and (expected == "any" or str(payload.get("mock")).lower() == expected) else 1)' "$base_url" "$expected_mock" > "$OUT_DIR/qwen_health.json" 2>/dev/null; then
      ready=1
      break
    fi
    if [[ -n "$qwen_pid" ]] && ! kill -0 "$qwen_pid" 2>/dev/null; then
      echo "Qwen service exited early" >&2
      tail -n 120 "$OUT_DIR/qwen_service.log" >&2 || true
      exit 1
    fi
    sleep 1
  done
  if [[ "$ready" != "1" ]]; then
    echo "Qwen service did not become healthy: $endpoint" >&2
    tail -n 120 "$OUT_DIR/qwen_service.log" >&2 || true
    exit 1
  fi
}

start_qwen_if_needed() {
  if [[ -n "$QWEN_ENDPOINT" ]]; then
    wait_for_qwen "$QWEN_ENDPOINT" "any"
    return
  fi
  QWEN_ENDPOINT="http://127.0.0.1:$QWEN_PORT/v1/chat/completions"
  case "$QWEN_MODE" in
    mock)
      QWEN_VLA_PORT="$QWEN_PORT" "$ROOT/scripts/run_qwen_vla_service.sh" --mock > "$OUT_DIR/qwen_service.log" 2>&1 &
      qwen_pid="$!"
      wait_for_qwen "$QWEN_ENDPOINT" "true"
      ;;
    real)
      QWEN_VLA_PORT="$QWEN_PORT" "$ROOT/scripts/run_qwen_vla_service.sh" > "$OUT_DIR/qwen_service.log" 2>&1 &
      qwen_pid="$!"
      wait_for_qwen "$QWEN_ENDPOINT" "false"
      ;;
    external)
      echo "PARKSIM_VIS_QWEN_MODE=external requires PARKSIM_VIS_QWEN_ENDPOINT" >&2
      exit 1
      ;;
    *)
      echo "Unsupported PARKSIM_VIS_QWEN_MODE: $QWEN_MODE" >&2
      exit 1
      ;;
  esac
}

encode_mode() {
  local mode="$1"
  local run_dir="$OUT_DIR/$mode"
  local frames_dir="$run_dir/frames"
  local frame_count
  frame_count="$(find "$frames_dir" -maxdepth 1 -name 'frame_*.png' | wc -l)"
  if (( frame_count < 2 )); then
    echo "$mode produced too few visualizer frames: $frame_count" >&2
    exit 1
  fi
  ffmpeg -hide_banner -loglevel error -y -framerate "$VIDEO_FPS" -i "$frames_dir/frame_%06d.png" \
    -vf "scale=1280:-2:flags=lanczos" -c:v libx264 -pix_fmt yuv420p "$run_dir/${mode}.mp4"
  ffmpeg -hide_banner -loglevel error -y -framerate "$VIDEO_FPS" -i "$frames_dir/frame_%06d.png" \
    -vf "fps=$GIF_FPS,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" "$run_dir/${mode}.gif"
  echo "$mode frames=$frame_count mp4=$run_dir/${mode}.mp4 gif=$run_dir/${mode}.gif"
}

encode_side_by_side() {
  local aligned_dir="$OUT_DIR/aligned_rule_vs_qwen_vla"
  PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" -m parksim.vla.align_visualizer_frames \
    --left "$OUT_DIR/rule_based" \
    --right "$OUT_DIR/qwen_vla" \
    --out-dir "$aligned_dir" \
    --dt "$ALIGN_DT" \
    --panel-width "$ALIGN_PANEL_WIDTH"
  ffmpeg -hide_banner -loglevel error -y -framerate "$VIDEO_FPS" -i "$aligned_dir/frame_%06d.png" \
    -c:v libx264 -pix_fmt yuv420p "$OUT_DIR/rule_vs_qwen_vla.mp4"
  ffmpeg -hide_banner -loglevel error -y -framerate "$VIDEO_FPS" -i "$aligned_dir/frame_%06d.png" \
    -vf "fps=$GIF_FPS,scale=1280:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" "$OUT_DIR/rule_vs_qwen_vla.gif"
}


controlled_ego_done() {
  local log_dir="$1"
  python3 - "$log_dir" <<'EARLY_STOP_CHECK'
import json
import sys
from pathlib import Path

log_dir = Path(sys.argv[1])
completed_fallback = False
for summary_path in sorted(log_dir.glob("vehicle_*_summary.json")):
    try:
        payload = json.loads(summary_path.read_text())
    except Exception:
        continue
    if payload.get("completed") is not True:
        continue
    if payload.get("is_controlled_ego") is True:
        raise SystemExit(0)
    if payload.get("vehicle_id") == 1:
        completed_fallback = True
if completed_fallback:
    raise SystemExit(0)
raise SystemExit(1)
EARLY_STOP_CHECK
}

wait_for_early_stop() {
  local sim_pid="$1"
  local log_dir="$2"
  if [[ "$EARLY_STOP" != "1" ]]; then
    return 1
  fi
  while kill -0 "$sim_pid" 2>/dev/null; do
    if controlled_ego_done "$log_dir" >/dev/null 2>&1; then
      echo "early_stop controlled ego completed -> $log_dir"
      cleanup_simulation_processes
      return 0
    fi
    sleep "$EARLY_STOP_POLL_SECONDS"
  done
  return 1
}

run_mode() {
  local mode="$1"
  local run_dir="$OUT_DIR/$mode"
  local frames_dir="$run_dir/frames"
  local log_dir="$run_dir/logs"
  local sim_log="$run_dir/simulator.log"
  local vis_log="$run_dir/visualizer.log"
  mkdir -p "$frames_dir" "$log_dir"
  echo "recording $mode visualizer -> $run_dir"

  cleanup_ros_processes
  PYTHONPATH="$DLP_ROOT${PYTHONPATH:+:$PYTHONPATH}" ros2 run parksim visualizer_node.py --ros-args \
    -p dlp_path:="$DLP_PATH" \
    -p timer_period:="$VIS_TIMER_PERIOD" \
    -p record_frames:=true \
    -p record_dir:="$frames_dir" \
    -p record_prefix:=frame \
    -p record_every_n:="$RECORD_EVERY_N" \
    > "$vis_log" 2>&1 &
  visualizer_pid="$!"
  sleep "$VIS_START_DELAY"
  if ! kill -0 "$visualizer_pid" 2>/dev/null; then
    echo "$mode visualizer exited early" >&2
    tail -n 160 "$vis_log" >&2 || true
    exit 1
  fi

  set +e
  local early_stop_triggered=0
  PYTHONPATH="$DLP_ROOT${PYTHONPATH:+:$PYTHONPATH}" timeout "$DURATION" ros2 run parksim simulator_node.py --ros-args \
    -p spawn_controlled_ego:=true \
    -p controlled_ego_agent_type:="$mode" \
    -p controlled_ego_spawn_time:="$SPAWN_TIME" \
    -p controlled_ego_spot_index:="$SPOT_INDEX" \
    -p controlled_ego_blocks_entrance:="$CONTROLLED_EGO_BLOCKS_ENTRANCE" \
    -p random_seed:="$RANDOM_SEED" \
    -p background_mode:="$BACKGROUND_MODE" \
    -p spawn_entering:="$SPAWN_ENTERING" \
    -p spawn_exiting:="$SPAWN_EXITING" \
    -p log_path:="$log_dir" \
    -p qwen_endpoint:="$QWEN_ENDPOINT" \
    -p qwen_timeout:="$QWEN_TIMEOUT" \
    > "$sim_log" 2>&1 &
  local sim_pid="$!"
  if wait_for_early_stop "$sim_pid" "$log_dir"; then
    early_stop_triggered=1
  fi
  wait "$sim_pid"
  local status="$?"
  if [[ "$early_stop_triggered" == "1" ]]; then
    status=0
  fi
  set -e
  sleep "$VIS_POST_ROLL"
  if [[ -n "$visualizer_pid" ]] && kill -0 "$visualizer_pid" 2>/dev/null; then
    kill "$visualizer_pid" 2>/dev/null || true
    wait "$visualizer_pid" 2>/dev/null || true
  fi
  cleanup_ros_processes
  if [[ "$status" != "0" && "$status" != "124" ]]; then
    echo "$mode simulator failed with exit status $status" >&2
    tail -n 160 "$sim_log" >&2 || true
    exit "$status"
  fi
  encode_mode "$mode"
}

start_qwen_if_needed
run_mode rule_based
run_mode qwen_vla
encode_side_by_side

PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" -m parksim.vla.compare_results "$OUT_DIR" --root "$ROOT"

cat >> "$OUT_DIR/summary.md" <<'EOF_SUMMARY'

## Visualizer Videos

- `rule_based/rule_based.mp4`: baseline policy video rendered by `visualizer_node.py`.
- `rule_based/rule_based.gif`: baseline policy GIF rendered by `visualizer_node.py`.
- `qwen_vla/qwen_vla.mp4`: Qwen-VLA policy video rendered by `visualizer_node.py`.
- `qwen_vla/qwen_vla.gif`: Qwen-VLA policy GIF rendered by `visualizer_node.py`.
- `rule_vs_qwen_vla.mp4`: side-by-side comparison video aligned by visualizer `sim_time`.
- `rule_vs_qwen_vla.gif`: side-by-side comparison GIF aligned by visualizer `sim_time`.
- `aligned_rule_vs_qwen_vla/alignment.json`: simulation-time alignment metadata.
- Early-stop is controlled by `PARKSIM_VIS_EARLY_STOP`; side-by-side frames remain aligned by visualizer `sim_time`.
EOF_SUMMARY

PYTHONPATH="$ROOT/python${PYTHONPATH:+:$PYTHONPATH}" "$PYTHON_BIN" -m parksim.vla.video_manifest "$OUT_DIR"

echo "visualizer_video_out_dir=$OUT_DIR"
find "$OUT_DIR" -maxdepth 2 \( -name '*.mp4' -o -name '*.gif' -o -name 'summary.md' -o -name 'metrics.csv' -o -name 'video_manifest.json' \) -print | sort
