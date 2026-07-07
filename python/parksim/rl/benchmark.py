import argparse
import time

import numpy as np

from parksim.rl.core import ParkSimCoreConfig, ParkSimCoreEnv


def benchmark_headless(num_envs: int, steps: int, num_agents: int):
    envs = [ParkSimCoreEnv(ParkSimCoreConfig(num_agents=num_agents, seed=i)) for i in range(num_envs)]
    for env in envs:
        env.reset()
    start = time.perf_counter()
    transitions = 0
    for _ in range(steps):
        for env in envs:
            actions = {agent_id: np.zeros(2, dtype=np.float32) for agent_id in env.agent_ids}
            env.step(actions)
            transitions += len(actions)
    elapsed = time.perf_counter() - start
    return {
        "num_envs": num_envs,
        "num_agents": num_agents,
        "steps_per_env": steps,
        "transitions": transitions,
        "elapsed_sec": elapsed,
        "transitions_per_sec": transitions / max(elapsed, 1e-9),
    }


def benchmark_multi_agent(num_envs: int, steps: int, num_agents: int):
    return benchmark_headless(num_envs=num_envs, steps=steps, num_agents=max(num_agents, 2))


def main():
    parser = argparse.ArgumentParser(description="Benchmark ParkSim headless RL throughput.")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--num-agents", type=int, default=1)
    parser.add_argument("--steps", type=int, default=1000)
    args = parser.parse_args()
    result = benchmark_headless(args.num_envs, args.steps, args.num_agents)
    for key, value in result.items():
        print(f"{key}: {value}")


if __name__ == "__main__":
    main()
