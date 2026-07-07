# ParkSim RL Interfaces

This package keeps reinforcement-learning rollout headless and reserves ROS for
evaluation and visualization.

## Training path

- `ParkSimCoreEnv` runs pure Python reset/step without ROS, GUI, subprocesses,
  topics, or services.
- `ParkSimParkingEnv` wraps the core as a single-agent Gymnasium environment.
- `ParkSimParallelParkingEnv` is added on the multi-agent branch.
- Use `PARKSIM_DATA_ROOT` to point at private ParkSim prior files when real DLP
  scenarios are needed. Private pickle/data/model files are intentionally not
  tracked by Git.

## Evaluation path

- `RLPolicyAgent` inherits the ParkSim `AbstractAgent` interface and can be
  placed back into the ROS/GUI simulator.
- `vehicle.launch.py` accepts `agent_type:=rule_based|rl_policy`.
- Learned checkpoints are loaded from `rl_policy_path` or `PARKSIM_POLICY_PATH`
  and are not committed to GitHub.

## Optional RL dependencies

Install optional RL wrappers with:

```bash
cd python
pip install -e ".[rl]"
```

Torch checkpoint loading is optional:

```bash
cd python
pip install -e ".[torch-policy]"
```

The core headless environment itself only requires NumPy and existing ParkSim
types.

## Expert data

`ExpertTrajectoryDataset` builds a common transition format for IRL:

```python
from parksim.rl.expert_data import ExpertTrajectoryDataset

dataset = ExpertTrajectoryDataset(data_root="/path/to/priorFiles", num_agents=4)
episodes = dataset.load()
features = dataset.feature_matrix()
```

When no private data root is passed, the dataset can generate a tiny synthetic
fixture for tests. Real DLP/ParkSim data remains local and is not tracked by Git.
