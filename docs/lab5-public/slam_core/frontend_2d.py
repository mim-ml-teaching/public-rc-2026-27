"""The frontend: it drives the (simulated) robot through the warehouse and produces what a real
robot would hand to the estimator: the odometry between consecutive poses and the sightings of
the tags, both with noise. It also knows the truth, which only the plots use."""

import math
import random
from typing import List, Sequence, Tuple

from .geometry import observe, relative_motion
from .models import Observation, Odometry, Pose2D
from .world import DEFAULT_WAYPOINTS, Cell, Warehouse


class WarehouseFrontend:
    def __init__(
        self,
        world: Warehouse,
        sigma_odom_xy: float = 0.02,  # odometry noise per step: metres ...
        sigma_odom_theta: float = math.radians(1.0),  # ... and radians
        sigma_range: float = 0.05,  # scanner noise: metres ...
        sigma_bearing: float = math.radians(1.0),  # ... and radians
        sensor_range: float = 4.0,  # the tags further away than this are not seen
        sensor_fov: float = math.pi,  # half the field of view: the scanner looks all around
        step: float = 0.5,  # the robot logs a pose and takes measurements every `step` metres
        seed: int = 1,
    ):
        self.step = step
        self.world = world
        self.sigma_odom_xy = sigma_odom_xy
        self.sigma_odom_theta = sigma_odom_theta
        self.sigma_range = sigma_range
        self.sigma_bearing = sigma_bearing
        self.sensor_range = sensor_range
        self.sensor_fov = sensor_fov
        self.rng = random.Random(seed)

    def run(self, waypoints: Sequence[Cell] = DEFAULT_WAYPOINTS) -> Tuple[List[Pose2D], List[Odometry], List[Observation]]:
        """Drive the route. Returns the true poses, the noisy odometry and the noisy observations."""
        true_poses = self.world.drive(waypoints, step=self.step)
        odometry = []
        for i, (a, b) in enumerate(zip(true_poses, true_poses[1:])):
            dx, dy, dtheta = relative_motion(a.x, a.y, a.theta, b.x, b.y, b.theta)
            odometry.append(
                Odometry(
                    step=i,
                    dx=dx + self.rng.gauss(0, self.sigma_odom_xy),
                    dy=dy + self.rng.gauss(0, self.sigma_odom_xy),
                    dtheta=dtheta + self.rng.gauss(0, self.sigma_odom_theta),
                )
            )
        observations = []
        for t, pose in enumerate(true_poses):
            for lm_id, (lx, ly) in self.world.landmarks.items():
                r, bearing = observe(pose.x, pose.y, pose.theta, lx, ly)
                if r > self.sensor_range or abs(bearing) > self.sensor_fov:
                    continue
                if not self.world.line_of_sight((pose.x, pose.y), (lx, ly)):
                    continue  # a shelf is in the way
                observations.append(
                    Observation(
                        pose_index=t,
                        landmark_id=lm_id,
                        range=r + self.rng.gauss(0, self.sigma_range),
                        bearing=bearing + self.rng.gauss(0, self.sigma_bearing),
                    )
                )
        return true_poses, odometry, observations
