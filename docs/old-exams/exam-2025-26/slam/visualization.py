import math
from typing import List, Optional

import matplotlib.pyplot as plt
from typing import Tuple


def plot_results_2d(
    true_poses: List[Tuple],
    true_landmarks: List[Tuple],
    odometry_measurements: List[Tuple],
    optimized_poses: List[Tuple],
    optimized_landmarks: List[Tuple],
    initial_pose: Tuple = ((0, 0), 0.0),
    save_path: Optional[str] = "slam_result.png",
    plot_headings: bool = True,
) -> None:
    """
    Plots the ground truth, dead reckoning (from noisy odometry), and optimized results.
    """
    plt.figure(figsize=(10, 6))

    # 1. Plot True Data
    xs = [p[0] for p in true_poses]
    ys = [p[1] for p in true_poses]
    thetas = [p[2] for p in true_poses]
    plt.plot(
        xs, ys, "g-o", label="Ground Truth Poses", markersize=10, linewidth=2, zorder=1
    )

    # Landmarks
    if true_landmarks:
        lx, ly = zip(*true_landmarks)
        plt.scatter(
            lx, ly, c="g", marker="*", s=200, label="Ground Truth Landmarks", zorder=10
        )

    if plot_headings:
        plt.quiver(
            xs,
            ys,
            [0.2 * math.cos(t) for t in thetas],
            [0.2 * math.sin(t) for t in thetas],
            angles="xy",
            scale_units="xy",
            scale=1,
            color="g",
            alpha=0.8,
            zorder=1,
            width=0.004,
        )

    # 2. Plot Dead Reckoning (Odometry Integration)

    current_x, current_y = initial_pose[0]
    current_theta = initial_pose[1]
    dr_xs = [current_x]
    dr_ys = [current_y]
    dr_thetas = [current_theta]

    for odom in odometry_measurements:
        delta_x = odom[0]
        delta_y = odom[1]
        delta_theta = odom[2]

        rotated_dx = (
            math.cos(current_theta) * delta_x - math.sin(current_theta) * delta_y
        )
        rotated_dy = (
            math.sin(current_theta) * delta_x + math.cos(current_theta) * delta_y
        )

        current_x += rotated_dx
        current_y += rotated_dy
        current_theta += delta_theta
        dr_xs.append(current_x)
        dr_ys.append(current_y)
        dr_thetas.append(current_theta)

    plt.plot(
        dr_xs, dr_ys, "r--x", label="Dead Reckoning (Noisy Odom)", alpha=0.6, zorder=2
    )

    if plot_headings:
        plt.quiver(
            dr_xs,
            dr_ys,
            [0.2 * math.cos(t) for t in dr_thetas],
            [0.2 * math.sin(t) for t in dr_thetas],
            angles="xy",
            scale_units="xy",
            scale=1,
            color="r",
            alpha=0.6,
            zorder=2,
            width=0.004,
        )

    # 3. Plot Optimized Data
    optimized_poses = optimized_poses.reshape(-1, 3)
    plt.plot(
        optimized_poses[:, 0],
        optimized_poses[:, 1],
        "b-s",
        label="Optimized Poses",
        markersize=6,
        zorder=3,
    )

    if optimized_landmarks:
        ox, oy = zip(*optimized_landmarks)
        plt.scatter(
            ox,
            oy,
            c="b",
            marker="X",
            s=150,
            label="Optimized Landmarks",
            alpha=0.8,
            zorder=11,
        )

    if plot_headings:
        plt.quiver(
            optimized_poses[:, 0],
            optimized_poses[:, 1],
            [0.2 * math.cos(t) for t in optimized_poses[:, 2]],
            [0.2 * math.sin(t) for t in optimized_poses[:, 2]],
            angles="xy",
            scale_units="xy",
            scale=1,
            color="b",
            alpha=0.8,
            zorder=3,
            width=0.004,
        )

    plt.title("2D SLAM Results")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.grid(True, linestyle="--", alpha=0.6)
    plt.legend()
    plt.axis("equal")  # Keep aspect ratio to judge distances correctly

    if save_path:
        print(f"Saving plot to {save_path}...")
        plt.savefig(save_path)
        print("Plot saved.")
    else:
        plt.show()
