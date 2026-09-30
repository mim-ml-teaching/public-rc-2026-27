from typing import List, Tuple

import numpy as np
from scipy import optimize

# This weight should work reasonably well, there is no need to change it.
SENSOR_WEIGHT = 10000.0


def optimize_slam(
    initial_poses_guess: List,
    initial_landmarks_guess: List,
    measured_movements: List,
    sensors_measurements: List,
    ground_truth_initial_pose: Tuple,  # We assume we have the access to the ground truth of starting position
    odometry_noise_std: float = 1.0,
) -> optimize.OptimizeResult:
    # Prepare the initial flat parameter vector
    # We optimize over poses and landmarks simultaneously, so they have to be packed into a single, flat numpy array
    # Parameters: [pose_0, pose_1, ..., pose_N, lm1_x, lm1_y, lm2_x, lm2_y, ...]
    num_poses = len(initial_poses_guess)
    initial_poses_guess = np.array([
        [p[0], p[1], p[2]] for p in initial_poses_guess
    ]).flatten()
    initial_landmarks_guess = np.array([
        [lm[0], lm[1]] for lm in initial_landmarks_guess
    ]).flatten()
    initial_params = np.concatenate((initial_poses_guess, initial_landmarks_guess))

    all_ids = sorted(sensors_measurements.keys())
    id_mapping = {landmark_id: idx for idx, landmark_id in enumerate(all_ids)}

    # Optimize
    result = optimize.minimize(
        _cost_function,
        initial_params,
        args=(
            num_poses,
            measured_movements,
            sensors_measurements,
            odometry_noise_std,
            id_mapping,
            ground_truth_initial_pose,
        ),
        method="Powell",
    )

    return result


def _cost_function(
    params: np.ndarray,
    num_poses: int,
    measured_movements: List,
    measurements: List,
    odometry_noise_std: float,
    id_mapping: dict,
    ground_truth_initial_pose: Tuple,
) -> float:
    """
    Calculates the error for a given set of parameters (2D pose and landmark estimation).
    """
    # Unpack parameters
    poses = params[: num_poses * 3].reshape(-1, 3)
    landmark_coords = params[num_poses * 3 :].reshape(-1, 2)

    # Reconstruct landmarks list [(x,y), ...]
    landmarks = []
    for lm in landmark_coords:
        landmarks.append((lm[0], lm[1]))

    # Avoid division by zero
    odom_weight = 1.0 / (odometry_noise_std**2) if odometry_noise_std > 1e-9 else 1.0
    sensor_weight = SENSOR_WEIGHT

    # 1. Movement Penalty (Odometry Error)
    # For each odometry measurement, we calculate the expected movement based on the current and next pose, and compare to the measured movement.
    movement_penalty = 0.0
    for i in range(len(poses) - 1):
        # State at i
        x1, y1, theta1 = poses[i]
        # State at i+1
        x2, y2, theta2 = poses[i + 1]
        # Measurement
        dx_meas = measured_movements[i][0]
        dy_meas = measured_movements[i][1]
        dtheta_meas = measured_movements[i][2]

        # Calculate the actual movement in the frame of pose i
        dx_global = x2 - x1
        dy_global = y2 - y1
        dtheta_global = theta2 - theta1

        # Rotate global difference by -theta1 to get local difference
        c, s = np.cos(-theta1), np.sin(-theta1)
        dx_local = c * dx_global - s * dy_global
        dy_local = s * dx_global + c * dy_global

        # Angle difference
        dtheta_diff = dtheta_global - dtheta_meas
        # Normalize to [-pi, pi]
        dtheta_diff = np.arctan2(np.sin(dtheta_diff), np.cos(dtheta_diff))

        movement_penalty += (
            (dx_local - dx_meas) ** 2 + (dy_local - dy_meas) ** 2 + (dtheta_diff) ** 2
        )

    movement_penalty *= odom_weight
    # 2. Observation Penalty (Sensor Error)
    # For each pose calculate expected distances and angles to all landmarks and compare to measurements
    distance_penalty = 0
    angle_penalty = 0

    ### TODO: Compute sensor model error.
    # Iterate over key/values in the 'measurements' dictionary.
    # The key is the 'landmark_id'. Use id_mapping[landmark_id] to retrieve the estimated position from 'landmarks' list.
    # The value is a list of measurements. For each measurement:
    #   1. Retrieve the robot pose using 'pose_idx'.
    #   2. Calculate expected distance/angle from robot to landmark.
    #   3. Add squared errors to penalties.

    for landmark_id, landmark_measurements in measurements.items():
        landmark_guess = landmarks[id_mapping[landmark_id]]

        # landmark guess is a tuple (x, y) of guessed position of landmark with id landmark_id
        # landmark_measurements is a list of measurements for this landmark
        # Each measurement is a dict with keys:
        #   - "pose_idx": index of the robot pose when the measurement was taken
        #   - "distance_angle": tuple (measured_distance, measured_angle)

    ### END TODO

    distance_penalty *= sensor_weight
    angle_penalty *= sensor_weight

    # 3. Prior (Anchor first pose)
    # This prevents the whole world from shifting arbitrarily
    # We can weight this heavily to ensure it sticks
    (gt_x, gt_y), gt_theta = ground_truth_initial_pose
    prior = (
        (poses[0][0] - gt_x) ** 2
        + (poses[0][1] - gt_y) ** 2
        + (poses[0][2] - gt_theta) ** 2
    ) * 1000.0

    cost = movement_penalty + prior + distance_penalty + angle_penalty
    return cost
