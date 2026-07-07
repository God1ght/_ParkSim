from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any, Callable, Dict, Optional

import numpy as np


class PolicyAdapter(ABC):
    @abstractmethod
    def predict(self, obs: Dict[str, np.ndarray], deterministic: bool = True) -> np.ndarray:
        raise NotImplementedError


class ConstantPolicyAdapter(PolicyAdapter):
    def __init__(self, action=None):
        self.action = np.asarray(action if action is not None else [0.0, 0.0], dtype=np.float32)

    def predict(self, obs: Dict[str, np.ndarray], deterministic: bool = True) -> np.ndarray:
        return self.action.copy()


class CallablePolicyAdapter(PolicyAdapter):
    def __init__(self, policy_fn: Callable[[Dict[str, np.ndarray]], Any]):
        self.policy_fn = policy_fn

    def predict(self, obs: Dict[str, np.ndarray], deterministic: bool = True) -> np.ndarray:
        return np.asarray(self.policy_fn(obs), dtype=np.float32)


class LinearNpzPolicyAdapter(PolicyAdapter):
    """Tiny no-framework policy loader for smoke tests and portable evaluation."""

    def __init__(self, path: str):
        payload = np.load(path)
        self.weights = np.asarray(payload["weights"], dtype=np.float32)
        self.bias = np.asarray(payload.get("bias", np.zeros(self.weights.shape[0])), dtype=np.float32)

    def predict(self, obs: Dict[str, np.ndarray], deterministic: bool = True) -> np.ndarray:
        flat = flatten_observation(obs)
        return np.tanh(self.weights @ flat + self.bias).astype(np.float32)


class TorchPolicyAdapter(PolicyAdapter):
    def __init__(self, path: str, device: str = "cpu"):
        try:
            import torch
        except ImportError as exc:  # pragma: no cover - optional dependency.
            raise ImportError("TorchPolicyAdapter requires torch.") from exc
        self.torch = torch
        self.device = device
        try:
            self.model = torch.jit.load(path, map_location=device)
        except Exception:
            self.model = torch.load(path, map_location=device)
        if hasattr(self.model, "eval"):
            self.model.eval()

    def predict(self, obs: Dict[str, np.ndarray], deterministic: bool = True) -> np.ndarray:
        flat = flatten_observation(obs)
        with self.torch.no_grad():
            tensor = self.torch.as_tensor(flat, dtype=self.torch.float32, device=self.device).unsqueeze(0)
            output = self.model(tensor)
            if isinstance(output, tuple):
                output = output[0]
            return output.squeeze(0).detach().cpu().numpy().astype(np.float32)


def load_policy_adapter(
    path: Optional[str] = None,
    policy_fn: Optional[Callable[[Dict[str, np.ndarray]], Any]] = None,
) -> PolicyAdapter:
    if policy_fn is not None:
        return CallablePolicyAdapter(policy_fn)
    if not path:
        return ConstantPolicyAdapter()
    policy_path = Path(path).expanduser()
    if not policy_path.exists():
        raise FileNotFoundError(f"Policy checkpoint not found: {policy_path}")
    if policy_path.suffix == ".npz":
        return LinearNpzPolicyAdapter(str(policy_path))
    return TorchPolicyAdapter(str(policy_path))


def flatten_observation(obs: Dict[str, Any]) -> np.ndarray:
    pieces = []
    for key in sorted(obs.keys()):
        value = obs[key]
        if isinstance(value, dict):
            pieces.append(flatten_observation(value))
        else:
            pieces.append(np.asarray(value, dtype=np.float32).reshape(-1))
    return np.concatenate(pieces).astype(np.float32) if pieces else np.zeros(0, dtype=np.float32)
