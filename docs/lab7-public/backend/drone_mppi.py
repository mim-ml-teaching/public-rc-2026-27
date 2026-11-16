"""Drone-specific adapter between the generic MPPI optimizer and Genesis."""

from dataclasses import replace

import torch

from mppi import MPPI, MPPIConfig
from rewards import DroneRewardEnv

DRONE_MPPI_CONFIG = MPPIConfig(horizon=60, num_samples=512, iterations=3, temperature=0.01)


class DroneMPPI:
    """Replan motor RPMs using a separate batched copy of the live environment.

    Actions are normalized RPM offsets: zero is hover, -1 is stopped. The live
    scene is never touched by speculative rollouts. The score is negative sum
    of the environment's racing and hover rewards, including its terminal masks.
    """

    def __init__(
        self, live: DroneRewardEnv, config: MPPIConfig | None = None, *, noise_std=0.025, seed=0
    ):
        if live.n_envs != 1:
            raise ValueError("DroneMPPI needs one live drone; num_samples sets the rollout batch")
        config = config or DRONE_MPPI_CONFIG
        self.live = live
        # Preserve student overrides as well as the reward weights in the model.
        self.rollouts = type(live)(
            replace(
                live.config,
                n_envs=config.num_samples,
                show_viewer=False,
                render=False,
                validate_actions=False,
            ),
            live.course,
            reward_config=live.reward_config,
        )
        self.optimizer = MPPI(
            self.rollout_cost,
            action_low=[-1.0] * 4,
            action_high=[live.config.max_rpm / live.hover_rpm - 1] * 4,
            noise_std=[noise_std] * 4,
            action_default=[0.0] * 4,
            config=config,
            device=live.device,
            seed=seed,
        )

    def rollout_cost(self, state, actions):
        self.rollouts.set_state(state)
        cost = torch.zeros(actions.shape[0], device=self.live.device)
        for t in range(actions.shape[1]):
            rpm = self.rollouts.hover_rpm * (1 + actions[:, t])
            _, reward, _ = self.rollouts.step(rpm)
            cost -= reward
        return cost

    def __call__(self, observation=None):
        state = self.live.get_state().select(0)
        action = self.optimizer.command(state)
        return (self.live.hover_rpm * (1 + action))[None]

    def reset(self):
        """Discard the previous plan when starting a new live episode."""
        self.optimizer.reset()
