"""Part 1, the frontend: a robot drives along one aisle, to the end and back. On both sides of
the aisle stand the shelf posts, each with a tag. The robot measures how far it moved (odometry,
noisy) and, for every post within range, how far ahead of it the post is along the aisle
(the along-aisle offset: a camera sees the tag, so it knows where it is in the robot's frame;
in 1D we keep only the forward component). Everything is a number on a line."""

import random
from typing import Dict, List, Tuple

from .models import Observation1D


class AisleFrontend:
    def __init__(
        self,
        length: float = 12.0,  # the aisle, in metres
        post_spacing: float = 2.0,  # a post every 2 m, on both sides
        half_width: float = 1.0,  # the posts stand 1 m left and right of the robot's line
        step: float = 1.0,  # the robot moves 1 m per step
        sigma_odom: float = 0.08,  # odometry noise per step (m)
        sigma_sensor: float = 0.05,  # sensor noise (m)
        sensor_range: float = 2.5,  # posts further away are not seen
        seed: int = 1,
    ):
        self.length = length
        self.half_width = half_width
        self.step = step
        self.sigma_odom = sigma_odom
        self.sigma_sensor = sigma_sensor
        self.sensor_range = sensor_range
        self.rng = random.Random(seed)
        # the posts: id -> (x along the aisle, y = which side)
        xs = [i * post_spacing for i in range(int(length / post_spacing) + 1)]
        self.posts: Dict[int, Tuple[float, float]] = {}
        for x in xs:
            self.posts[len(self.posts)] = (x, half_width)
            self.posts[len(self.posts)] = (x, -half_width)

    def run(self) -> Tuple[List[float], List[float], List[Observation1D]]:
        """Drive to the end of the aisle and back. Returns the true positions, the noisy odometry
        (one displacement per step) and the noisy observations."""
        n = int(self.length / self.step)
        true_x = [0.5]
        for d in [self.step] * n + [-self.step] * n:
            true_x.append(true_x[-1] + d)
        odometry = [b - a + self.rng.gauss(0, self.sigma_odom) for a, b in zip(true_x, true_x[1:])]
        observations = []
        for t, x in enumerate(true_x):
            for post_id, (px, _) in self.posts.items():
                if abs(px - x) <= self.sensor_range:
                    observations.append(Observation1D(t, post_id, px - x + self.rng.gauss(0, self.sigma_sensor)))
        return true_x, odometry, observations
