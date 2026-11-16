"""Student task: racing and hovering rewards on top of the drone simulation."""

import math
from dataclasses import dataclass, fields

import torch

from backend.drone_env import DroneGateEnv, Transition


@dataclass(frozen=True)
class RewardConfig:
    """Non-negative weights; set a weight to zero to disable its component."""

    progress_weight: float = 1.0
    time_weight: float = 0.1
    smoothness_weight: float = 0.01
    gate_bonus: float = 1.0
    finish_bonus: float = 5.0
    failure_penalty: float = 25.0
    yaw_rate_weight: float = 0.005
    hover_velocity_weight: float = 0.2
    hover_angular_velocity_weight: float = 0.05

    def __post_init__(self):
        for field in fields(self):
            value = getattr(self, field.name)
            if not math.isfinite(value) or value < 0:
                raise ValueError(f"{field.name} must be finite and non-negative")


class DroneRewardEnv(DroneGateEnv):
    """Edit the component methods here; physics and gate bookkeeping are inherited.

    Each method returns a signed, weighted tensor of shape (n_envs,).
    Rewards are evaluated per physics substep, not per controller command.
    """

    def __init__(self, config=None, course=None, *, reward_config=None):
        self.reward_config = reward_config or RewardConfig()
        super().__init__(config, course)

    def compute_reward(self, step: Transition):
        """The total reward is the sum of these independently editable components."""
        return {
            "progress": self.reward_progress(step),
            "time": self.reward_time(step),
            "smoothness": self.reward_smoothness(step),
            "gate": self.reward_gate(step),
            "finish": self.reward_finish(step),
            "failure": self.reward_failure(step),
            "yaw_rate": self.reward_yaw_rate(step),
            "hover_velocity": self.reward_hover_velocity(step),
            "hover_angular_velocity": self.reward_hover_angular_velocity(step),
        }

    def reward_progress(self, step):
        """Reward decreasing distance to the gate targeted BEFORE this substep."""
        ### TODO
        # Return (n_envs,) weighted change in distance toward step.gate_center while racing,
        # using the same gate before/after the step and replacing non-finite progress with zero.
        # Invalid simulations earn a finite failure penalty, not a NaN score.
        raise NotImplementedError  # Your code here
        ### END TODO

    def reward_time(self, step):
        """Penalize simulated time while racing, including the final crossing."""
        return -self.reward_config.time_weight * self.config.physics_dt * step.racing.float()

    def reward_smoothness(self, step):
        """Penalize a motor-command change once; holding the command costs zero."""
        ### TODO
        # Return (n_envs,) rewards as the negative weighted sum of squared motor-RPM
        # changes normalized by self.hover_rpm, with zero reward outside step.racing.
        raise NotImplementedError  # Your code here
        ### END TODO

    def reward_gate(self, step):
        """One bonus per valid, ordered crossing."""
        return self.reward_config.gate_bonus * step.passed.float()

    def reward_finish(self, step):
        """One bonus for crossing the last gate."""
        return self.reward_config.finish_bonus * step.finished.float()

    def reward_failure(self, step):
        """One penalty for collision, a missed gate, or non-finite physical state."""
        return -self.reward_config.failure_penalty * step.failed.float()

    def reward_yaw_rate(self, step):
        """Optionally discourage spin about the drone's body Z while racing."""
        ### TODO
        # Return (n_envs,) negative weighted squared yaw rates times physics_dt while
        # racing, with zero for non-finite rates, disabled weight, or non-racing rows.
        raise NotImplementedError  # Your code here
        ### END TODO

    def reward_hover_velocity(self, step):
        """After the last gate, penalize linear speed without a position target."""
        velocity = torch.nan_to_num(step.velocity, nan=0, posinf=0, neginf=0)
        penalty = -self.reward_config.hover_velocity_weight * velocity.square().sum(-1)
        return torch.where(step.hovering, penalty * self.config.physics_dt, 0.0)

    def reward_hover_angular_velocity(self, step):
        """After the last gate, penalize rotation about all axes."""
        omega = torch.nan_to_num(step.angular_velocity, nan=0, posinf=0, neginf=0)
        penalty = -self.reward_config.hover_angular_velocity_weight * omega.square().sum(-1)
        return torch.where(step.hovering, penalty * self.config.physics_dt, 0.0)
