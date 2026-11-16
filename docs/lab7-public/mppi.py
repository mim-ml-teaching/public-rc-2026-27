"""Model Predictive Path Integral control, independent of any robot/simulator.

The only model-specific input is rollout_cost(state, actions). It simulates a
batch of candidate sequences (samples, horizon, action_dim) from the SAME state
and returns one total cost per sample. Lower cost is better. The state may be
a tensor, a simulator snapshot, or any object understood by that callback.

No gradients, dynamics equations, gates, or drone-specific gains live here.
"""

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from typing import Any

import torch


@dataclass(frozen=True)
class MPPIConfig:
    horizon: int = 30
    num_samples: int = 256
    iterations: int = 2
    temperature: float = 0.01

    def __post_init__(self):
        if self.horizon < 1 or self.num_samples < 2 or self.iterations < 1:
            raise ValueError("horizon/iterations must be positive and num_samples >= 2")
        if not math.isfinite(self.temperature) or self.temperature <= 0:
            raise ValueError("temperature must be finite and positive")


class MPPI:
    def __init__(
        self,
        rollout_cost: Callable[[Any, torch.Tensor], torch.Tensor],
        *,
        action_low: Sequence[float] | torch.Tensor,
        action_high: Sequence[float] | torch.Tensor,
        noise_std: Sequence[float] | torch.Tensor,
        config: MPPIConfig | None = None,
        action_default: Sequence[float] | torch.Tensor | None = None,
        device: str | torch.device = "cpu",
        seed: int = 0,
    ):
        self.config = config or MPPIConfig()
        self.rollout_cost = rollout_cost
        self.device = torch.device(device)
        self.low = torch.as_tensor(action_low, dtype=torch.float32, device=self.device)
        self.device = self.low.device  # Resolve e.g. "cuda" to the actual device index.
        self.high = torch.as_tensor(action_high, dtype=torch.float32, device=self.device)
        self.noise_std = torch.as_tensor(noise_std, dtype=torch.float32, device=self.device)
        if self.low.ndim != 1 or self.low.numel() == 0:
            raise ValueError("Action bounds must be nonempty vectors")
        if self.low.shape != self.high.shape or self.low.shape != self.noise_std.shape:
            raise ValueError("Action bounds and noise_std must have the same shape")
        if not bool(torch.isfinite(torch.stack((self.low, self.high, self.noise_std))).all()):
            raise ValueError("Action bounds and noise_std must be finite")
        if bool((self.low >= self.high).any()) or bool((self.noise_std <= 0).any()):
            raise ValueError("Each lower bound must be below its upper bound; noise_std > 0")
        default = (self.low + self.high) / 2 if action_default is None else action_default
        self.default = torch.as_tensor(default, dtype=torch.float32, device=self.device)
        if self.default.shape != self.low.shape or not bool(torch.isfinite(self.default).all()):
            raise ValueError("action_default must be a finite vector matching the bounds")
        if bool(((self.default < self.low) | (self.default > self.high)).any()):
            raise ValueError("action_default must lie inside the bounds")
        # A local generator makes experiments repeatable without reseeding the
        # simulator. MPS uses CPU random sampling; CUDA samples on its device.
        rng_device = "cpu" if self.device.type == "mps" else self.device
        self.generator = torch.Generator(device=rng_device).manual_seed(seed)
        self.current_plan = self.default.repeat(self.config.horizon, 1)
        self.diagnostics = {}

    @torch.inference_mode()
    def reset(self, sequence=None):
        """Forget the old plan, or supply a new (horizon, action_dim) warm start."""
        if sequence is None:
            self.current_plan = self.default.repeat(self.config.horizon, 1)
        else:
            sequence = torch.as_tensor(sequence, dtype=torch.float32, device=self.device)
            if sequence.shape != self.current_plan.shape or not bool(torch.isfinite(sequence).all()):
                raise ValueError("Warm start must be finite and match (horizon, action_dim)")
            self.current_plan = sequence.clamp(self.low, self.high).clone()
        self.diagnostics = {}

    def sample_actions(self):
        """Sample bounded action sequences around the current plan: (N, H, A)."""
        ### TODO
        # Return bounded (samples, horizon, action_dim) sequences sampled around
        # self.current_plan with self.generator and self.noise_std, keeping sample 0 unperturbed.
        # Always evaluate the current plan too.
        raise NotImplementedError  # Your code here
        ### END TODO

    def compute_weights(self, costs):
        """Give lower-cost rollouts exponentially more weight; weights sum to one."""
        ### TODO
        # Return (samples,) weights from a stable softmax of negative costs / temperature,
        # assigning zero to non-finite costs and raising if no valid candidate remains.
        # Before softmax, subtract the best score to keep the exponential numerically stable.
        raise NotImplementedError  # Your code here
        ### END TODO

    def update_plan(self, candidates, weights):
        """Average the simulated commands, weighted by how well they performed."""
        return (weights[:, None, None] * candidates).sum(dim=0).clamp(self.low, self.high)

    @torch.inference_mode()
    def plan(self, state):
        """Optimize a sequence without advancing the real system or shifting it."""
        ### TODO
        # Repeat for self.config.iterations:
        # 1. Sample candidate action sequences.
        # 2. Calculate rollout costs from the same initial state.
        # 3. Compute normalized weights.
        # 4. Update self.current_plan with the weighted plan.
        raise NotImplementedError  # Your code here
        ### END TODO

        # Provided logging for the last optimization iteration.
        self.diagnostics = {
            "costs": costs.clone(),
            "weights": weights,
            "best_cost": costs[torch.argmax(weights)],
            "effective_samples": 1 / weights.square().sum(),
        }
        return self.current_plan.clone()

    @torch.inference_mode()
    def command(self, state):
        """Plan, return the first action, and shift the remainder for warm start.

        Apply this action for ONE model step, then call command with the new
        observed state. The discarded final slot is replaced by action_default.
        """
        sequence = self.plan(state)
        action = sequence[0].clone()
        self.current_plan = torch.cat((sequence[1:], self.default[None]), dim=0)
        return action
