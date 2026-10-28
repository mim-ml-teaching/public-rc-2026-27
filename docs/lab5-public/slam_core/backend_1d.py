"""Part 1, the backend: the unknowns are the robot's position at every step and the position of
every post along the aisle, packed into one vector [x_0 ... x_T, p_1 ... p_K]. Both models are
linear here (a displacement is a difference of positions, an offset is a difference of positions),
so least squares finds the answer in one step."""

from typing import Dict, List, Sequence, Tuple

import numpy as np
from scipy import optimize

from .models import Observation1D


class AisleBackend:
    def __init__(self, sigma_odom: float, sigma_sensor: float, sigma_prior: float = 1e-3):
        self.sigma_odom = sigma_odom
        self.sigma_sensor = sigma_sensor
        self.sigma_prior = sigma_prior

    @staticmethod
    def dead_reckoning(start: float, odometry: Sequence[float]) -> np.ndarray:
        return start + np.concatenate([[0.0], np.cumsum(odometry)])

    def initial_guess(self, start, odometry, observations, post_ids) -> Tuple[np.ndarray, np.ndarray]:
        """Positions by dead reckoning; every post where it was first seen from."""
        x = self.dead_reckoning(start, odometry)
        index = {p: k for k, p in enumerate(post_ids)}
        posts = np.full(len(post_ids), np.nan)
        for obs in observations:
            k = index[obs.landmark_id]
            if np.isnan(posts[k]):
                posts[k] = x[obs.pose_index] + obs.offset
        return x, posts

    def residuals(self, params, n_poses, start, odom, obs_pose, obs_post, obs_z):
        """[prior, odometry (one per step), observations (one per sighting)], each divided by its sigma."""
        x = params[:n_poses]
        posts = params[n_poses:]
        r_prior = (x[0] - start) / self.sigma_prior
        ### TODO ###
        # odometry, one residual per step: the displacement the estimate implies minus the measured one
        # (odom[t] is the measured displacement from x[t] to x[t + 1])
        # observations, one residual per sighting: where post obs_post[i] should appear as seen from
        # pose obs_pose[i] (its offset along the aisle), minus where it did (obs_z[i])
        raise NotImplementedError  # your code here
        ### END TODO ###
        return np.concatenate([[r_prior], r_odom, r_obs])

    def optimize(self, start: float, odometry: Sequence[float], observations: Sequence[Observation1D]):
        post_ids = sorted({o.landmark_id for o in observations})
        index = {p: k for k, p in enumerate(post_ids)}
        n_poses = len(odometry) + 1
        x0, posts0 = self.initial_guess(start, odometry, observations, post_ids)
        odom = np.array(odometry)
        obs_pose = np.array([o.pose_index for o in observations])
        obs_post = np.array([index[o.landmark_id] for o in observations])
        obs_z = np.array([o.offset for o in observations])
        result = optimize.least_squares(
            self.residuals, np.concatenate([x0, posts0]), args=(n_poses, start, odom, obs_pose, obs_post, obs_z)
        )
        x = result.x[:n_poses]
        posts: Dict[int, float] = {p: float(result.x[n_poses + k]) for p, k in index.items()}
        return x, posts, result
