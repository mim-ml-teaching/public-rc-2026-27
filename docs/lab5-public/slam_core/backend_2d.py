"""The backend: given the odometry and the observations, find the poses and the landmark
positions that explain them best. The unknowns are packed into one vector x; the cost is
the sum of squared, weighted residuals; scipy's least_squares does the linearise-solve-iterate."""

from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from scipy import optimize
from scipy.sparse import lil_matrix

from .geometry import compose, landmark_from_observation, observe, relative_motion, wrap_angle
from .models import Observation, Odometry, Pose2D


class WarehouseBackend:
    def __init__(
        self,
        sigma_odom_xy: float,
        sigma_odom_theta: float,
        sigma_range: float,
        sigma_bearing: float,
        sigma_prior: float = 1e-3,  # how sure we are about the starting pose: very
        use_sparsity: bool = True,
    ):
        self.sigma_odom_xy = sigma_odom_xy
        self.sigma_odom_theta = sigma_odom_theta
        self.sigma_range = sigma_range
        self.sigma_bearing = sigma_bearing
        self.sigma_prior = sigma_prior
        self.use_sparsity = use_sparsity

    # ---- the unknowns as one vector -------------------------------------------------------------
    @staticmethod
    def pack(poses: np.ndarray, landmarks: np.ndarray) -> np.ndarray:
        """poses: (N, 3) array of (x, y, theta); landmarks: (M, 2) array. Returns the vector x."""
        return np.concatenate([poses.ravel(), landmarks.ravel()])

    @staticmethod
    def unpack(x: np.ndarray, n_poses: int) -> Tuple[np.ndarray, np.ndarray]:
        poses = x[: 3 * n_poses].reshape(n_poses, 3)
        landmarks = x[3 * n_poses :].reshape(-1, 2)
        return poses, landmarks

    # ---- the initial guess ------------------------------------------------------------------------
    @staticmethod
    def initial_guess(
        start: Pose2D, odometry: Sequence[Odometry], observations: Sequence[Observation], landmark_ids: Sequence[int]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Poses by dead reckoning; every landmark where it was first seen from (from the dead-reckoned pose)."""
        poses = [(start.x, start.y, start.theta)]
        for o in odometry:
            poses.append(compose(*poses[-1], o.dx, o.dy, o.dtheta))
        poses = np.array(poses)
        index = {lm: k for k, lm in enumerate(landmark_ids)}
        landmarks = np.full((len(landmark_ids), 2), np.nan)
        for obs in observations:  # in order of time, so the first sighting wins
            k = index[obs.landmark_id]
            if np.isnan(landmarks[k, 0]):
                x, y, theta = poses[obs.pose_index]
                landmarks[k] = landmark_from_observation(x, y, theta, obs.range, obs.bearing)
        return poses, landmarks

    # ---- the residuals ----------------------------------------------------------------------------
    def residuals(
        self,
        x: np.ndarray,
        n_poses: int,
        start: Pose2D,
        odom: np.ndarray,  # (N-1, 3): the measured (dx, dy, dtheta) from pose i to pose i+1
        obs_pose: np.ndarray,  # (K,): the pose index of every observation
        obs_lm: np.ndarray,  # (K,): the landmark index of every observation
        obs_z: np.ndarray,  # (K, 2): the measured (range, bearing)
    ) -> np.ndarray:
        """All residuals, each divided by the noise sigma of its measurement, as one vector:
        [prior (3), odometry (3 per step), observations (2 per sighting)].
        A residual is (what the measurement should be for the current estimate) - (what it was)."""
        poses, landmarks = self.unpack(x, n_poses)

        # 1. The prior: the robot started where we said it started.
        r_prior = np.array([poses[0, 0] - start.x, poses[0, 1] - start.y, wrap_angle(poses[0, 2] - start.theta)])
        r_prior /= self.sigma_prior

        ### TODO ###
        # 2. Odometry: the motion between consecutive estimated poses vs the measured one.
        # 3. Observations: where the estimated landmark should appear from the estimated pose vs where it did.
        raise NotImplementedError  # your code here
        ### END TODO ###

        return np.concatenate([r_prior, r_odom.ravel(), r_obs.ravel()])

    def jacobian_sparsity(self, n_poses: int, n_landmarks: int, obs_pose: np.ndarray, obs_lm: np.ndarray):
        """Which unknowns each residual depends on: a 0/1 matrix, one row per residual (in the
        order of `residuals()`), one column per unknown (in the order of `pack()`). Only the
        pattern matters: it lets the solver compute the Jacobian with a handful of residual
        evaluations instead of one per unknown. Returning None is allowed: the solver still
        works, it is just slower."""
        ### TODO (part 3) ###
        # scipy.sparse.lil_matrix is a convenient way to fill such a matrix
        return None  # replace by your code
        ### END TODO ###

    # ---- the solve ---------------------------------------------------------------------------------
    def optimize(
        self,
        start: Pose2D,
        odometry: Sequence[Odometry],
        observations: Sequence[Observation],
        landmark_ids: Optional[Sequence[int]] = None,
        initial: Optional[Tuple[np.ndarray, np.ndarray]] = None,
        verbose: int = 0,
    ) -> Tuple[np.ndarray, Dict[int, Tuple[float, float]], optimize.OptimizeResult]:
        """Returns the estimated poses (N, 3), the estimated landmarks {id: (x, y)} and scipy's result."""
        if landmark_ids is None:
            landmark_ids = sorted({o.landmark_id for o in observations})
        index = {lm: k for k, lm in enumerate(landmark_ids)}
        n_poses = len(odometry) + 1
        if initial is None:
            initial = self.initial_guess(start, odometry, observations, landmark_ids)
        x0 = self.pack(*initial)

        odom = np.array([[o.dx, o.dy, o.dtheta] for o in odometry]).reshape(-1, 3)
        obs_pose = np.array([o.pose_index for o in observations], dtype=int)
        obs_lm = np.array([index[o.landmark_id] for o in observations], dtype=int)
        obs_z = np.array([[o.range, o.bearing] for o in observations]).reshape(-1, 2)

        sparsity = self.jacobian_sparsity(n_poses, len(landmark_ids), obs_pose, obs_lm) if self.use_sparsity else None
        result = optimize.least_squares(
            self.residuals,
            x0,
            args=(n_poses, start, odom, obs_pose, obs_lm, obs_z),
            jac_sparsity=sparsity,
            method="trf",
            x_scale="jac",
            # with the sparsity pattern, every step is solved iteratively (LSMR); its default tolerance is too
            # loose for a loop closure, where the whole route has to bend at once, and the solve then crawls
            tr_options={"atol": 1e-12, "btol": 1e-12},
            verbose=verbose,
        )
        poses, landmarks = self.unpack(result.x, n_poses)
        poses = poses.copy()
        poses[:, 2] = wrap_angle(poses[:, 2])
        return poses, {lm: (float(landmarks[k, 0]), float(landmarks[k, 1])) for lm, k in index.items()}, result


def dead_reckoning(start: Pose2D, odometry: Sequence[Odometry]) -> np.ndarray:
    """The poses obtained by adding up the odometry, nothing else. (N, 3)."""
    poses = [(start.x, start.y, start.theta)]
    for o in odometry:
        poses.append(compose(*poses[-1], o.dx, o.dy, o.dtheta))
    return np.array(poses)
