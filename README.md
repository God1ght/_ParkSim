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

The protocol skeleton is stored in `python/parksim/vla/benchmark_protocol.json`.
