# ParkSim VLA Prototype

This package implements a high-level VLA decision layer for ParkSim. In the TR-C paper workflow, Qwen-VLA is treated as a cloud fleet coordinator for automated parking vehicles, while replay/rule-random/mixed vehicles represent human-driven background traffic with partially hidden intent. The model chooses task-level actions such as waiting, selecting a parking spot, cruising to a spot, rerouting, parking, or exiting. Low-level steering and acceleration remain handled by the existing ParkSim rule-based planner and Stanley controller.

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

Each Qwen-controlled vehicle uses the shared cloud endpoint. The default runtime packet is `ParkSim-Qwen-VLA-Fleet-Decision-v1`, which asks Qwen to return strict JSON with `fleet_decisions`. Each item contains `vehicle_id`, `action_id`, `target_spot_index`, `priority`, `reason_code`, `reason`, and `confidence`. The older `ParkSim-Qwen-VLA-Decision-v1` single-vehicle packet is retained for compatibility and smoke tests.

The fleet decision packet contains `automated_vehicle_ids`, one `automated_vehicles` entry per AV needing a high-level decision, parking-lot-level state, and optional BEV evidence. A centralized ROS fleet coordinator creates one synchronized epoch for all eligible AVs. It first waits for the simulator to confirm that simulation time is frozen, then collects vehicle contexts, obtains and shields the joint decision, waits for every addressed AV to confirm application at the same simulation time, and only then resumes simulation. Qwen wall-clock response time is retained only in raw JSONL for engineering audit and is excluded from traffic-performance metrics. Replay/rule vehicles remain non-Qwen human-like traffic.

The required structured fields are:

- `decision_contract`: output schema, hard constraints, and the source of spot availability.
- `world_model`: whether occupancy is ready, number of spots, available spot count, and blocked/unknown spot count.
- `ego`: controlled vehicle id, task, pose, speed, current target, braking state, and wait target.
- `candidate_spots`: nearest selectable parking spaces with `status=available`, coordinates, and distance.
- `candidate_assignment_bundles`: executable spot-route-wait candidates with `action_id`, `spot_index`, `route_id`, `route_strategy`, `path_length_m`, `eta_s`, `expected_wait_s`, `conflict_risk`, `conflict_vehicle_count`, and `bundle_cost`. Lower `bundle_cost` is preferred only after hard constraints are satisfied.
- `selection_objective`: the system-level objective used by Qwen and bundle-aware baselines: reduce route length, expected wait, dynamic conflict risk, and conflict vehicle count under partial observability.
- `blocked_nearby_spots`: nearby occupied or unknown spots with `central_occupied`, dynamic occupancy, and reasons, included only to explain why they are not valid choices.
- `nearby_vehicles`: nearby vehicle kinematics and observable behavior. Background task/progress/destination are hidden by default unless `reveal_background_intents_to_vla=true`.
- `central_occupancy` and `effective_occupancy`: raw simulator occupancy and safety-filtered occupancy.
- `protocol_version`, `prompt_version`, `output_schema`, `reason_codes`, and `hard_constraints`: the reproducible VLA decision contract.
- `valid_action_ids` and `valid_actions`: the only action ids Qwen is allowed to choose.
- `bev_image_path`: rendered BEV context where available spots, occupied spots, other vehicles, and ego are drawn.

The action enumerator and safety shield both use the same `spot_status` layer. A spot is selectable only when central occupancy is known, central occupancy is false, and no dynamic vehicle is occupying that spot. Unknown or occupied spots are excluded from `valid_actions`, and a Qwen output that names such a spot is rejected before execution.

Entry parking tasks expose only parking-progress actions. `CRUISE_TO_EXIT` is generated only for exiting vehicles or active unpark/exit task semantics, preventing Qwen from satisfying a parking episode by leaving the lot.

Bundle-aware baseline agents are available as `bundle_risk_aware`, `conflict_aware_bundle`, `min_bundle_cost`, `reservation_bundle`, `rolling_horizon_bundle`, `centralized_min_cost`, and `oracle_intent_bundle`. They use the same candidate bundle interface and safety shield as Qwen-VLA, which makes paper comparisons focus on high-level assignment/path selection rather than different low-level controllers. `oracle_intent_bundle` reveals hidden task/progress/reference-path information and should be reported only as an upper bound, not as a deployable policy.

Audit decision logs with `python -m parksim.vla.decision_audit <benchmark-or-log> --out-dir <audit-dir> --strict`. This is the paper-facing check for protocol version, prompt version, valid action membership, target consistency, reason code coverage, shield rejections, and unsafe applied parking spots.

Use `python -m parksim.vla.paper_gate <suite-dir> --profile pilot --strict` to verify that a suite is structurally valid for pilot evidence. Use `--profile paper` to check paper-scale coverage requirements such as all baseline agents, at least three seeds, full background/density coverage, statistical report outputs, real Qwen health, and zero unsafe applied actions by the reference Qwen-VLA agent.

Use `--profile trc` for Transportation Research Part C oriented evidence. This profile is CSV/JSON-first: it checks fleet efficiency, operational safety, cloud coordination, mixed-human traffic, synchronous-decision integrity, real Qwen health, decision audits, video evidence, and open-science manifests. It does not treat Qwen response latency as a performance metric and does not require a TeX manuscript; manuscript writing should be based on the CSV/JSON analysis outputs.

Fleet epochs are event-triggered by a newly registered AV, an idle high-level task boundary, or a stalled/yielding maneuver that is eligible for replanning. A simulation-time watchdog provides bounded state staleness; the TR-C protocol uses 30 s. CSV outputs distinguish event-triggered, watchdog, actionable, and empty epochs. Qwen wall-clock latency remains audit-only, while the simulator stays frozen until all addressed vehicles acknowledge applying the decision at the same simulation time.

Before launching the full 630-episode matrix, run the single-seed medium-density 3600 s calibration:

```bash
./scripts/run_vla_trc_calibration.sh
```

The calibration writes only CSV/JSON/Markdown analysis artifacts. It does not edit manuscript or TeX files. On a host shared with MAPPO/MAHAN/HAN/IPPO training, queue the protected workflow instead:

```bash
nohup ./scripts/run_vla_post_training_calibration.sh > post_training_calibration.log 2>&1 &
```

The queued workflow waits for both the protected training regex and GPU compute process list to remain empty, then runs offline smoke tests, rebuilds ROS, validates the synchronous decision barrier with the mock service, and finally starts the real-Qwen 3600 s calibration. Its `status.json` and calibration output path are stored under `experiments/qwen_vla_jobs/` and `experiments/qwen_vla_paper_suite/`.

## Policy comparison artifacts

Run a reproducible headless comparison between the baseline rule-based policy and the Qwen-VLA high-level policy:

```bash
./scripts/build_parksim_ros.sh --cmake-clean-cache
PARKSIM_COMPARE_QWEN_MODE=real ./scripts/run_policy_comparison.sh
```

Use `PARKSIM_COMPARE_QWEN_MODE=mock` for a fast CI-style check, `real` to start the local Qwen service, or `external` with `PARKSIM_COMPARE_QWEN_ENDPOINT` for an already running service. The script writes `metrics.json`, `metrics.csv`, `summary.md`, `trajectories.png`, and `metrics.png` under `experiments/qwen_vla_comparison/<timestamp>/`.

## Long-Horizon Human-Mixed Evaluation

Use the long-horizon entrypoint when the simulator should represent a realistic human-machine mixed parking lot rather than a short single-ego benchmark:

```bash
PARKSIM_LONG_QWEN_MODE=real ./scripts/run_vla_long_horizon_suite.sh
```

This wraps `run_vla_benchmark.sh` with `traffic_flow_mode=human_mixed_long_horizon`, disables ego early-stop by default, restores a configurable fraction of original static obstacle spots as rule-based exiting vehicles, and generates many entering/exiting tasks over a long horizon. Rule/replay background vehicle intentions are ground truth for evaluation logs only; VLA inputs expose pose, speed, braking, waiting, occupancy, and valid actions, but hide destination, planned route, and task profile by default.

Key overrides:

- `PARKSIM_LONG_DURATION` / `PARKSIM_LONG_HORIZON_SECONDS`: wall-clock simulator timeout and scenario horizon.
- `PARKSIM_LONG_SPAWN_ENTERING` / `PARKSIM_LONG_SPAWN_EXITING`: requested long-horizon demand volume.
- `PARKSIM_LONG_RESTORE_OBSTACLES_AS_EXIT_VEHICLES`: convert initial occupied obstacle spots into departure tasks.
- `PARKSIM_LONG_HIDDEN_INTENT_FRACTION`: fraction of rule-agent intentions hidden from the VLA state.
- `PARKSIM_LONG_MAX_CONCURRENT_BACKGROUND_VEHICLES`: cap on scheduled background traffic.

Each run writes `traffic_schedule.json`, `traffic_events.jsonl`, vehicle traces with `vehicle_role` and `intent_observable`, and system-level conflict metrics such as `system_near_miss_event_count`, `trajectory_conflict_event_count`, and `mixed_intent_conflict_event_count`. Fleet-control runs also write `automated_vehicle_count`, `cloud_served_vehicle_count`, `human_like_vehicle_count`, `cloud_fleet_decision_count`, and `cloud_fleet_vehicle_decision_count` into `metrics.csv` and the aggregated paper CSV files. Default long-horizon agents now include `rule_based`, `greedy_nearest`, `greedy_shortest_path`, `risk_aware_rule`, `bundle_risk_aware`, `conflict_aware_bundle`, `reservation_bundle`, `rolling_horizon_bundle`, `centralized_min_cost`, `oracle_intent_bundle`, and `qwen_vla`.

Mixed/replay runs reserve every DLP replay vehicle ID before launching controlled or scheduled vehicles. Dynamic IDs are allocated above the replay-ID range, so a replay vehicle and an AV cannot share a ROS namespace or `vehicle_<id>_trace.jsonl`. Each benchmark also exports `trace_integrity_ok`, identity-conflict, time-regression, wall-time-regression, and kinematic-jump counts. Any nonzero integrity violation fails validation and is excluded from paper aggregation.

Cloud-controlled methods use a strict decision-then-advance barrier: the simulator is paused at the decision epoch, Qwen returns a fleet action set, the critic and system shield validate it, and every addressed AV acknowledges the action at the same simulator time before the next step. Qwen wall-clock response latency is audit-only and is excluded from all operational performance metrics. Paper suites additionally create `window_metrics.csv`, `window_paired_deltas.csv`, `critical_window_rank.csv`, and `window_metrics_manifest.json` under the report `critical_states` directory. The default 300 s half-open simulator-time windows yield 12 comparable windows for every 3600 s episode without double-counting boundary conflicts.


### Paper Baseline Interpretation

- `greedy_nearest` and `greedy_shortest_path` test myopic assignment rules.
- `risk_aware_rule`, `bundle_risk_aware`, and `conflict_aware_bundle` test local conflict-aware bundle scoring.
- `reservation_bundle` approximates reservation-based planning by penalizing near-term path conflicts and expected wait.
- `centralized_min_cost` is a deterministic centralized-cost proxy over global occupancy, nearby traffic load, and bundle cost.
- `oracle_intent_bundle` reveals hidden background task/progress/reference-path fields and should be interpreted as an upper-bound baseline under full intent observability.
- Qwen-VLA remains weight-frozen in this protocol; optimization is limited to state/action packet design, prompt contract, safety shielding, and candidate-bundle selection.

## Visualizer Video/GIF Comparison

Generate visualizer-node videos for the baseline rule-based policy and the Qwen-VLA policy:

```bash
./scripts/build_parksim_ros.sh --cmake-clean-cache
PARKSIM_VIS_QWEN_MODE=real ./scripts/run_visualizer_policy_videos.sh
```

Use `PARKSIM_VIS_QWEN_MODE=mock` for a fast pipeline check, `real` to start the local Qwen service, or `external` with `PARKSIM_VIS_QWEN_ENDPOINT` for an already running service. The script runs under `xvfb`, records frames from `visualizer_node.py`, writes `frame_times.jsonl` with each frame sim time, and writes per-policy MP4/GIF files plus simulation-time-aligned `rule_vs_qwen_vla.mp4` and `rule_vs_qwen_vla.gif` under `experiments/qwen_vla_visualizer_videos/<timestamp>/`. The side-by-side outputs are resampled by visualizer `sim_time`, not by Qwen wall-clock response time.

Runtime dependencies on `172.16.0.250` are `xvfb`, `xauth`, `ffmpeg`, Mesa GL packages, and `dearpygui`. DearPyGUI must run under `LIBGL_ALWAYS_SOFTWARE=1` in headless mode, which the script sets automatically.
