# ParkSim VLA Prototype

This package implements a high-level VLA decision layer for ParkSim. The model chooses task-level actions such as waiting, selecting a parking spot, cruising to a spot, rerouting, or parking. Low-level steering and acceleration remain handled by the existing ParkSim rule-based planner and Stanley controller.

The Qwen endpoint is optional for smoke testing. If it is unset or unavailable, the policy falls back to a deterministic valid action so that ROS integration can be tested before a local Qwen-VL service is deployed.

## Local Qwen Service

Run the service on `172.16.0.250` from the repo root:

```bash
./scripts/run_qwen_vla_service.sh
```

The service is OpenAI-compatible at:

```text
http://127.0.0.1:8000/v1/chat/completions
```

For a no-model health/smoke path:

```bash
./scripts/run_qwen_vla_service.sh --mock
```

To connect ParkSim to the local service:

```bash
ros2 launch parksim vehicle.launch.py \
  agent_type:=qwen_vla \
  qwen_endpoint:=http://127.0.0.1:8000/v1/chat/completions
```

Model cache defaults to `/media/step/data/models/huggingface`. Download the default 7B model with:

```bash
./scripts/download_qwen_vla_model.sh
```

Set `QWEN_MODEL_ID`, `QWEN_MODEL_CACHE`, `QWEN_VLA_PORT`, `QWEN_TORCH_DTYPE`, `QWEN_MAX_NEW_TOKENS`, and `HF_ENDPOINT` to override defaults. On `172.16.0.250`, official Hugging Face timed out during setup, so scripts default `HF_ENDPOINT` to `https://hf-mirror.com`. After the model is downloaded, `run_qwen_vla_service.sh` automatically prefers the latest local snapshot under `QWEN_MODEL_CACHE` and enables offline loading to avoid runtime network stalls. Set `QWEN_MODEL_LOCAL_ONLY=0` to force model-id loading, `QWEN_MODEL_LOCAL_ONLY=1` to require a local snapshot, or `QWEN_MODEL_LOCAL_SNAPSHOT=/abs/snapshot/path` to pin one snapshot.



## Qwen Decision Context

Each VLA decision uploads a versioned `ParkSim-Qwen-VLA-Decision-v1` decision packet plus an optional BEV image. Qwen must return only strict JSON with `action_id`, `target_spot_index`, `reason_code`, `reason`, and `confidence`.

The required structured fields are:

- `decision_contract`: output schema, hard constraints, and the source of spot availability.
- `world_model`: whether occupancy is ready, number of spots, available spot count, and blocked/unknown spot count.
- `ego`: controlled vehicle id, task, pose, speed, current target, braking state, and wait target.
- `candidate_spots`: nearest selectable parking spaces with `status=available`, coordinates, and distance.
- `blocked_nearby_spots`: nearby occupied or unknown spots with `central_occupied`, dynamic occupancy, and reasons, included only to explain why they are not valid choices.
- `nearby_vehicles`: nearby vehicle state and task/progress information.
- `central_occupancy` and `effective_occupancy`: raw simulator occupancy and safety-filtered occupancy.
- `protocol_version`, `prompt_version`, `output_schema`, `reason_codes`, and `hard_constraints`: the reproducible VLA decision contract.
- `valid_action_ids` and `valid_actions`: the only action ids Qwen is allowed to choose.
- `bev_image_path`: rendered BEV context where available spots, occupied spots, other vehicles, and ego are drawn.

The action enumerator and safety shield both use the same `spot_status` layer. A spot is selectable only when central occupancy is known, central occupancy is false, and no dynamic vehicle is occupying that spot. Unknown or occupied spots are excluded from `valid_actions`, and a Qwen output that names such a spot is rejected before execution.

Audit decision logs with `python -m parksim.vla.decision_audit <benchmark-or-log> --out-dir <audit-dir> --strict`. This is the paper-facing check for protocol version, prompt version, valid action membership, target consistency, reason code coverage, shield rejections, and unsafe applied parking spots.

Use `python -m parksim.vla.paper_gate <suite-dir> --profile pilot --strict` to verify that a suite is structurally valid for pilot evidence. Use `--profile paper` to check paper-scale coverage requirements such as all baseline agents, at least three seeds, full background/density coverage, statistical report outputs, real Qwen health, and zero unsafe applied actions by the reference Qwen-VLA agent.

## Policy comparison artifacts

Run a reproducible headless comparison between the baseline rule-based policy and the Qwen-VLA high-level policy:

```bash
./scripts/build_parksim_ros.sh --cmake-clean-cache
PARKSIM_COMPARE_QWEN_MODE=real ./scripts/run_policy_comparison.sh
```

Use `PARKSIM_COMPARE_QWEN_MODE=mock` for a fast CI-style check, `real` to start the local Qwen service, or `external` with `PARKSIM_COMPARE_QWEN_ENDPOINT` for an already running service. The script writes `metrics.json`, `metrics.csv`, `summary.md`, `trajectories.png`, and `metrics.png` under `experiments/qwen_vla_comparison/<timestamp>/`.

## Visualizer Video/GIF Comparison

Generate visualizer-node videos for the baseline rule-based policy and the Qwen-VLA policy:

```bash
./scripts/build_parksim_ros.sh --cmake-clean-cache
PARKSIM_VIS_QWEN_MODE=real ./scripts/run_visualizer_policy_videos.sh
```

Use `PARKSIM_VIS_QWEN_MODE=mock` for a fast pipeline check, `real` to start the local Qwen service, or `external` with `PARKSIM_VIS_QWEN_ENDPOINT` for an already running service. The script runs under `xvfb`, records frames from `visualizer_node.py`, writes `frame_times.jsonl` with each frame sim time, and writes per-policy MP4/GIF files plus simulation-time-aligned `rule_vs_qwen_vla.mp4` and `rule_vs_qwen_vla.gif` under `experiments/qwen_vla_visualizer_videos/<timestamp>/`. The side-by-side outputs are resampled by visualizer `sim_time`, not by Qwen wall-clock response time.

Runtime dependencies on `172.16.0.250` are `xvfb`, `xauth`, `ffmpeg`, Mesa GL packages, and `dearpygui`. DearPyGUI must run under `LIBGL_ALWAYS_SOFTWARE=1` in headless mode, which the script sets automatically.
