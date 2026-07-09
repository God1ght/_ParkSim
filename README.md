# ParkSim
![](https://img.shields.io/badge/language-python-blue)
![](https://img.shields.io/github/license/XuShenLZ/ParkSim)
![](https://img.shields.io/badge/ROS-foxy-red)

Vehicle simualtion and behavior prediction in parking lots.

Authors: Xu Shen (xu_shen@berkeley.edu), Alex Wong, Neelay Velingker, Matthew Lacayo, Nidhir Guggilla

## Install
1. Clone this repo
2. In the `/python` folder of this repo, do `pip install -e .` (A virtualenv is recommended)
3. If components of [DLP dataset](https://github.com/MPC-Berkeley/dlp-dataset) is needed, install the DLP package and request data according to the instructions there.
4. If [pytorch](https://pytorch.org/) is needed, please install the correct version based on your OS and hardware.
5. We use [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) for simulation. You would need that installed if you want to run vehicle simulation.

## [ParkPredict+](https://arxiv.org/abs/2204.10777)

ParkPredict+: Multimodal Intent and Motion Prediction for Vehicles in Parking Lots with CNN and Transformer

Authors: Xu Shen, Matthew Lacayo, Nidhir Guggilla, Francesco Borrelli

> **Note**: You don't need ROS for ParkPredict+.

<div align=center>
<img height="300" src="docs/multimodal_1.gif"/>  <img height="300" src="docs/multimodal_2.gif"/>
<img height="300" src="docs/multimodal_3.gif"/>  <img height="300" src="docs/multimodal_4.gif"/>
</div>

### Usage
See [this document](https://github.com/XuShenLZ/ParkSim/tree/main/python/parksim/trajectory_predict) for instruction.
1. A pre-trained intent prediction model can be [downloaded here](https://drive.google.com/file/d/1LVQJRQmjGfGchxhMRchiZRCjrlFDVch-/view?usp=sharing).
2. A pre-trained trajectory prediction model can be [downloaded here](https://drive.google.com/file/d/1c9KQXwFMRIYPJo1sXJKepoBcrEme_HxU/view?usp=sharing).

## ParkSim-VLA-Bench

This branch adds a reproducible high-level policy benchmark for Qwen-VLA style parking decisions. The benchmark keeps the low-level A*/Stanley/parking maneuver stack fixed and compares only the high-level action selector.

Available controlled ego agents:

- `rule_based`: original rule-based Stanley vehicle with fixed assigned spot.
- `greedy_nearest`: VLA-action baseline selecting the closest verified available spot.
- `greedy_shortest_path`: VLA-action baseline selecting the shortest A* route to a verified available spot.
- `risk_aware_rule`: VLA-action baseline using route length, estimated time, nearby-vehicle risk, and shield-compatible occupancy features.
- `qwen_vla`: Qwen policy client selecting one `action_id` from the same `valid_actions` schema.

Run a quick local benchmark after ROS build and DLP setup:

```bash
PARKSIM_BENCH_SEEDS="0" \
PARKSIM_BENCH_BACKGROUND_MODES="rule_random" \
PARKSIM_BENCH_AGENTS="rule_based greedy_nearest greedy_shortest_path risk_aware_rule qwen_vla" \
PARKSIM_BENCH_QWEN_MODE="mock" \
./scripts/run_vla_benchmark.sh
```

Outputs are written under `experiments/qwen_vla_benchmark/<timestamp>/`:

- `episodes.jsonl`: scenario, seed, agent, and log-path manifest.
- `metrics.csv` / `metrics.json`: episode-level reproducibility table.
- `summary.md` / `summary.json`: agent-level aggregate metrics.
- `preference_dataset.jsonl`: pairwise lower-is-better preference pairs for prompt optimization or offline preference tuning.
- `run_config.json`: benchmark configuration snapshot, including branch, commit, agents, seeds, and scenario factors.
- `validation.json` / `manifest.json`: completeness checks and artifact index for reproducible paper tables.

The metric table includes progress, timing, Qwen latency, shield/fallback counts, protocol compliance counts, and safety proxies such as minimum other-vehicle distance, near-miss events, collision-proxy events, TTC, malformed decisions, target mismatches, missing reason codes, and unsafe occupancy choices. Visualizer-video runs also write `video_manifest.json` to bind MP4/GIF files, frame counts, sim-time alignment, and metric files.

Audit Qwen-VLA decision logs after any benchmark or suite run with:

```bash
PYTHONPATH=python python3 -m parksim.vla.decision_audit experiments/qwen_vla_benchmark/<run> --out-dir experiments/qwen_vla_benchmark/<run>/audit --strict
```

The audit writes `decision_audit.json` and `decision_audit_failures.jsonl` with protocol-version, prompt-version, valid-action membership, target consistency, reason-code, shield, and unsafe-applied-spot checks.

The protocol skeleton is stored in `python/parksim/vla/benchmark_protocol.json`.

Controlled ego vehicles block the entrance spawn gate by default during benchmark and visualizer runs. This prevents random entering background vehicles from spawning at the same entrance pose as the controlled ego and turning spawn overlap into a false collision metric.
Cross-vehicle safety metrics synchronize traces by `wall_time` when available, so late-spawned background vehicles are not aligned to the ego vehicle by their per-node relative time.

Aggregate several validated benchmark directories into paper-ready tables:

```bash
PYTHONPATH=python python3 -m parksim.vla.paper_report \
  experiments/qwen_vla_paper_report/demo \
  --inputs experiments/qwen_vla_benchmark/20260709_124434 experiments/qwen_vla_benchmark/20260709_130221 \
  --reference-agent qwen_vla
```

The paper report writes `paper_summary.csv`, `paper_summary.md`, `paper_paired_deltas.csv`, `paper_paired_delta_summary.csv`, `paper_table.tex`, and `paper_report_manifest.json`.

Run a full paper-suite orchestration across modes, density levels, seeds, and agents:

```bash
PARKSIM_SUITE_SEEDS="0 1 2" \
PARKSIM_SUITE_BACKGROUND_MODES="rule_random mixed replay" \
PARKSIM_SUITE_DENSITY_CONFIGS="empty:0:0 light:1:0 balanced:2:2 dense:4:4" \
PARKSIM_SUITE_AGENTS="rule_based greedy_nearest greedy_shortest_path risk_aware_rule qwen_vla" \
PARKSIM_SUITE_QWEN_MODE="real" \
PARKSIM_SUITE_DURATION="120s" \
./scripts/run_vla_paper_suite.sh
```

Suite outputs are written under `experiments/qwen_vla_paper_suite/<timestamp>/` and include `suite_config.json`, `benchmarks.jsonl`, `suite_manifest.json`, per-condition benchmark directories, and final paper-report tables under `reports/paper_report/`.
For `replay` background mode, non-empty density configs are skipped by default because replay ignores random entering/exiting spawn density; set `PARKSIM_SUITE_REPLAY_ALL_DENSITIES=1` only if duplicate replay-density runs are intentionally needed.
The suite runner supports `PARKSIM_SUITE_RESUME=1` by default, so validated benchmark directories are reused instead of rerun. Use `PARKSIM_SUITE_DRY_RUN=1` to write a planned suite manifest without launching ROS, and `PARKSIM_SUITE_CONTINUE_ON_FAIL=1` to keep later conditions running after one benchmark fails while recording `failures.jsonl`.
