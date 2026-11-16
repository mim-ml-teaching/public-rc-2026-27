"""Fly a lab track with MPPI and save a video and trajectory plots."""

import argparse
import time

import genesis as gs
import torch

from backend.course import Course
from backend.drone_env import EnvConfig
from backend.drone_mppi import DroneMPPI
from backend.presets import PRESETS
from backend.runtime import FlightRecording
from rewards import DroneRewardEnv


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--track", choices=PRESETS, default="sanity")
    parser.add_argument("--samples", type=int, help="Override the track's parallel rollout count")
    parser.add_argument("--horizon", type=int, help="Override lookahead in 20 ms control steps")
    parser.add_argument("--iterations", type=int, help="Override MPPI updates per control step")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--substeps", type=int, choices=(1, 2, 4), default=2)
    parser.add_argument(
        "--backend",
        choices=("cpu", "gpu"),
        default="cpu",
        help="CPU or CUDA/ROCm GPU (Metal is not supported)",
    )
    args = parser.parse_args()
    if args.backend == "gpu" and not torch.cuda.is_available():
        parser.error("--backend gpu requires CUDA/ROCm; use --backend cpu. Metal is not supported.")
    preset = PRESETS[args.track]
    try:
        config = preset.mppi_config(
            samples=args.samples, horizon=args.horizon, iterations=args.iterations
        )
    except ValueError as error:
        parser.error(str(error))

    started = time.perf_counter()
    # Select explicitly: Genesis's generic GPU backend can fall back to Metal.
    backend = gs.cpu if args.backend == "cpu" else (gs.amdgpu if torch.version.hip else gs.cuda)
    gs.init(backend=backend, logging_level="warning", seed=args.seed)
    env = DroneRewardEnv(
        EnvConfig(
            render=True,
            physics_substeps=args.substeps,
            hover_after_finish=preset.hover_after_finish,
        ),
        Course.load_track(args.track),
    )
    controller = DroneMPPI(env, config, seed=args.seed)
    # Recording, camera, time limit, and final-hover checks are provided utilities.
    with FlightRecording(env, controller, preset, args, started) as recording:
        obs = env.observe()
        with torch.inference_mode():
            while not recording.complete:
                plan_started = time.perf_counter()
                rpm = controller(obs)
                planning_seconds = time.perf_counter() - plan_started
                obs, reward, events = env.step(rpm)
                recording.record(obs, reward, events, rpm, planning_seconds)


if __name__ == "__main__":
    main()
