"""Provided recording and reporting utilities, outside the student control loop."""

import json
import math
import time
from dataclasses import asdict
from pathlib import Path

import imageio.v2 as imageio
import numpy as np
import torch
from genesis.utils.geom import pos_lookat_up_to_T

from backend.drone_env import body_yaw_rate
from backend.plot_trajectory import save_trajectory_plots


class FlightRecording:
    """Record one live drone; stop on failure, task completion, or a time limit."""

    hover_seconds = 5.0

    def __init__(self, env, controller, preset, args, process_started):
        self.env = env
        self.controller = controller
        self.preset = preset
        self.args = args
        self.process_started = process_started
        self.steps = 0
        self.settled_steps = 0
        self.completed_task = False
        self.observations = []
        self.rewards = []
        self.rpms = []
        self.planning_seconds = []
        self.effective_samples = []
        self.reward_totals = {}
        self.episode_return = torch.zeros(1, device=env.device)
        config = controller.optimizer.config
        name = (
            f"{args.track}-n{config.num_samples}-h{config.horizon}-i{config.iterations}"
            f"-s{args.seed}-sub{args.substeps}-{args.backend}"
        )
        # Reserve a new directory rather than overwriting earlier experiments.
        for index in range(1, 1000000):
            directory = Path("outputs") / f"{name}-{index:03d}"
            try:
                directory.mkdir(parents=True)
                break
            except FileExistsError:
                continue
        else:
            raise RuntimeError("Could not allocate a recording directory")
        self.path = directory / "flight"

    def __enter__(self):
        self.writer = imageio.get_writer(str(self.path.with_suffix(".mp4")), fps=25)
        self._observe(self.env.observe())
        self.started = time.perf_counter()
        print(f"Track: {self.env.course.name}; outputs: {self.path.parent}", flush=True)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.writer.close()
        self.wall_seconds = time.perf_counter() - self.started
        if exc_type is None:
            self._save()
            if bool(self.env.failed[0]):
                raise SystemExit("Drone failed; inspect the saved video and trajectory")
            if not self.completed_task:
                raise SystemExit("Time limit reached before completing the flight and final hover")

    @property
    def complete(self):
        return (
            self.completed_task
            or bool(self.env.failed[0])
            or (self.steps >= round(self.preset.seconds / self.env.dt))
        )

    def _observe(self, obs):
        sample = {
            name: obs[name].cpu().numpy().copy()
            for name in ("position", "quaternion", "angular_velocity", "velocity")
        }
        sample["yaw_rate"] = (
            body_yaw_rate(obs["quaternion"], obs["angular_velocity"]).cpu().numpy().copy()
        )
        self.observations.append(sample)

    def record(self, obs, reward, events, rpm, planning_seconds):
        self.steps += 1
        self._observe(obs)
        self.rewards.append(reward.cpu().numpy().copy())
        self.rpms.append(rpm.cpu().numpy().copy())
        self.episode_return += reward
        self.planning_seconds.append(planning_seconds)
        self.effective_samples.append(
            float(self.controller.optimizer.diagnostics["effective_samples"])
        )
        for name, value in events["reward_terms"].items():
            self.reward_totals[name] = self.reward_totals.get(name, 0) + value
        self.settled_steps = self.settled_steps + 1 if bool(obs["hover_settled"][0]) else 0
        self.completed_task = bool(obs["finished"][0]) and (
            not self.env.config.hover_after_finish
            or (
                float(obs["hover_time"][0]) >= self.hover_seconds
                and self.settled_steps * self.env.dt >= 1.0
            )
        )
        if bool(events["passed_gate"][0]):
            print(
                f"t={float(obs['elapsed_time'][0]):.2f}s: passed gate "
                f"{int(obs['next_gate'][0])}/{len(self.env.course.gates)}",
                flush=True,
            )
        if (self.steps - 1) % 50 == 0:
            print(
                f"t={self.steps * self.env.dt:.2f}s; return={float(self.episode_return[0]):.3f}",
                flush=True,
            )
        if (self.steps - 1) % 2 == 0:
            self._follow_camera(obs)
            self.writer.append_data(self.env.camera.render()[0])

    def _follow_camera(self, obs):
        position = obs["position"][0].cpu().numpy()
        vx, vy = obs["velocity"][0, :2].cpu().tolist()
        if math.hypot(vx, vy) < 0.2:
            target_index = min(int(obs["next_gate"][0]), len(self.env.course.gates) - 1)
            target = self.env.course.gates[target_index].center
            vx, vy = target[0] - position[0], target[1] - position[1]
        speed = max(math.hypot(vx, vy), 1e-6)
        fx, fy = vx / speed, vy / speed
        camera_pos = position + [-1.2 * fx - 1.8 * fy, -1.2 * fy + 1.8 * fx, 0.9]
        lookat = position + [0.6 * fx, 0.6 * fy, 0.0]
        # Rebuild from world Z to avoid accumulating camera roll during turns.
        pose = pos_lookat_up_to_T(camera_pos, lookat, np.array([0.0, 0.0, 1.0]))
        self.env.camera.set_pose(transform=pose)

    def _save(self):
        env = self.env
        obs = env.observe()
        arrays = {
            name: np.stack([sample[name] for sample in self.observations])
            for name in self.observations[0]
        }
        yaw_rates = arrays["yaw_rate"]
        finish_time = float(obs["finish_time"][0])
        metrics = {
            "course": asdict(env.course),
            "time_limit": self.preset.seconds,
            "controller": "mppi",
            "num_envs": 1,
            "backend": self.args.backend,
            "physics_dt": env.config.physics_dt,
            "physics_substeps": env.config.physics_substeps,
            "control_dt": env.dt,
            "simulated_seconds": self.steps * env.dt,
            "wall_seconds_including_rendering": self.wall_seconds,
            "gates_passed": obs["next_gate"].tolist(),
            "finished": obs["finished"].tolist(),
            "failed": obs["failed"].tolist(),
            "finish_time": [finish_time if math.isfinite(finish_time) else None],
            "final_position": obs["position"].tolist(),
            "episode_return": self.episode_return.tolist(),
            "reward_terms": {name: value.tolist() for name, value in self.reward_totals.items()},
            "reward_config": asdict(env.reward_config),
            "yaw_rate_metrics": {
                "frame": "body Z",
                "units": "rad/s",
                "mean_abs": np.abs(yaw_rates).mean(0).tolist(),
                "rms": np.sqrt((yaw_rates**2).mean(0)).tolist(),
                "max_abs": np.abs(yaw_rates).max(0).tolist(),
            },
            "mean_planning_seconds": float(np.mean(self.planning_seconds)),
            "hover_after_finish": env.config.hover_after_finish,
            "hover_seconds_requested": self.hover_seconds,
            "hover_metrics": {
                "speed_m_s": torch.linalg.vector_norm(obs["velocity"], dim=-1).tolist(),
                "angular_speed_rad_s": torch.linalg.vector_norm(
                    obs["angular_velocity"], dim=-1
                ).tolist(),
                "hover_time": obs["hover_time"].tolist(),
                "settled": obs["hover_settled"].tolist(),
                "settled_for_seconds": [self.settled_steps * env.dt],
                "speed_tolerance_m_s": env.config.hover_speed_tolerance,
                "angular_speed_tolerance_rad_s": env.config.hover_angular_velocity_tolerance,
            },
            "mppi": {
                **asdict(self.controller.optimizer.config),
                "noise_std": 0.025,
                "seed": self.args.seed,
                "mean_effective_samples": float(np.mean(self.effective_samples)),
            },
        }
        np.savez_compressed(
            self.path.with_suffix(".npz"),
            **arrays,
            reward=np.asarray(self.rewards, dtype=np.float32),
            rpm=np.asarray(self.rpms, dtype=np.float32),
            dt=env.dt,
        )
        save_trajectory_plots(env.course, arrays["position"][:, 0], self.path.with_suffix(".png"))
        metrics["wall_seconds_since_start"] = time.perf_counter() - self.process_started
        self.path.with_suffix(".json").write_text(json.dumps(metrics, indent=2) + "\n")
        print(
            f"Passed {int(obs['next_gate'][0])}/{len(env.course.gates)} gates; "
            f"simulated {self.steps * env.dt:.2f}s; "
            f"elapsed {metrics['wall_seconds_since_start']:.1f}s.\n"
            f"Video, plots, and data: {self.path.parent.resolve()}",
            flush=True,
        )
