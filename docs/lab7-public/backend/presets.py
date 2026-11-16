"""Validated lab runs; settings shared by live and speculative environments."""

from dataclasses import dataclass

from mppi import MPPIConfig


@dataclass(frozen=True)
class Preset:
    num_samples: int
    horizon: int
    iterations: int
    seconds: float
    hover_after_finish: bool = False

    def mppi_config(self, *, samples=None, horizon=None, iterations=None):
        return MPPIConfig(
            num_samples=self.num_samples if samples is None else samples,
            horizon=self.horizon if horizon is None else horizon,
            iterations=self.iterations if iterations is None else iterations,
        )


PRESETS = {
    "sanity": Preset(256, 30, 2, 8),
    "straight": Preset(256, 40, 3, 8),
    "loop": Preset(1024, 60, 6, 25, hover_after_finish=True),
}
