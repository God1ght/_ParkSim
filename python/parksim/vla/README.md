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


## Policy comparison artifacts

Run a reproducible headless comparison between the baseline rule-based policy and the Qwen-VLA high-level policy:

```bash
./scripts/build_parksim_ros.sh --cmake-clean-cache
PARKSIM_COMPARE_QWEN_MODE=real ./scripts/run_policy_comparison.sh
```

Use `PARKSIM_COMPARE_QWEN_MODE=mock` for a fast CI-style check, `real` to start the local Qwen service, or `external` with `PARKSIM_COMPARE_QWEN_ENDPOINT` for an already running service. The script writes `metrics.json`, `metrics.csv`, `summary.md`, `trajectories.png`, and `metrics.png` under `experiments/qwen_vla_comparison/<timestamp>/`.
