import json
from typing import Tuple, Optional

import numpy as np

from navigation_simulator import NavigationSimulator


class ReplayNavigationSimulator(NavigationSimulator):
    """
    Replays a recorded trajectory and sensor measurements.

    The recording JSON should have the structure:
    {
        "meta": {"width": int, "height": int, "gps_freq": float, "gps_noise_std": float, "imu_noise_std": float},
        "steps": [
            {"dt": float, "true_pos": [px, py], "imu": [ax, ay], "gps": [px, py] | null},
            ...
        ]
    }

    Provides the same API as the live simulators:
        - run_step() -> (gps_reading, imu_reading, dt)
        - render() -> image with true position drawn
    """

    def __init__(self, recording_path: str):
        with open(recording_path, "r") as f:
            data = json.load(f)

        meta = data.get("meta", {})
        width = int(meta.get("width", 800))
        height = int(meta.get("height", 600))
        super().__init__(width=width, height=height)
        
        self.gps_freq = float(meta.get("gps_freq", 2.0))
        self.gps_noise_std = float(meta.get("gps_noise_std", 5.0))
        self.imu_noise_std = float(meta.get("imu_noise_std", 20.0))

        self.steps = data.get("steps", [])
        self._idx = 0

    def run_step(self) -> Tuple[Optional[np.ndarray], np.ndarray, float]:
        if self._idx >= len(self.steps):
            # End of replay; hold last state and provide zero dt
            return None, np.zeros(2, dtype=float), 0.0

        s = self.steps[self._idx]
        self._idx += 1

        dt = float(s.get("dt", 0.0))
        tp = np.array(s.get("true_pos", [self.true_pos[0], self.true_pos[1]]), dtype=float)
        imu = np.array(s.get("imu", [0.0, 0.0]), dtype=float)
        gps = s.get("gps", None)
        gps_arr = None if gps is None else np.array(gps, dtype=float)

        # Update true position
        self.true_pos = tp

        return gps_arr, imu, dt

