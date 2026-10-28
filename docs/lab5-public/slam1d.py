"""Part 1: SLAM in one aisle.

    uv run slam1d.py           run the drive, print the errors, show and save the plot
    uv run slam1d.py --play    the drive step by step, with whatever is implemented so far
"""

import sys

import matplotlib.pyplot as plt
import numpy as np

from slam_core.backend_1d import AisleBackend
from slam_core.frontend_1d import AisleFrontend
from slam_core.player import play_aisle
from slam_core.visualization import plot_aisle_results

SIGMA_ODOM = 0.08  # metres per step
SIGMA_SENSOR = 0.05  # metres


def main():
    frontend = AisleFrontend(sigma_odom=SIGMA_ODOM, sigma_sensor=SIGMA_SENSOR, seed=1)
    true_x, odometry, observations = frontend.run()
    print(f"{len(true_x)} steps, {len(observations)} sightings of {len(frontend.posts)} posts")

    backend = AisleBackend(sigma_odom=SIGMA_ODOM, sigma_sensor=SIGMA_SENSOR)
    start = true_x[0]  # the robot knows where it started
    if "--play" in sys.argv:
        play_aisle(frontend.posts, true_x, odometry, observations, backend, frontend.half_width)
        return
    dr_x = backend.dead_reckoning(start, odometry)
    est_x, est_posts, result = backend.optimize(start, odometry, observations)
    print(f"solved: {result.nfev} residual evaluations, final cost {2 * result.cost:.1f} for {len(result.fun)} residuals")

    true_x = np.array(true_x)
    for name, x in (("dead reckoning", dr_x), ("SLAM", est_x)):
        err = np.abs(x - true_x)
        print(f"{name:15s} position error: mean {err.mean():.3f} m, max {err.max():.3f} m, at the end {err[-1]:.3f} m")
    post_err = [abs(est_posts[k] - frontend.posts[k][0]) for k in est_posts]
    print(f"posts: mean error {np.mean(post_err):.3f} m, max {np.max(post_err):.3f} m")

    # the same drive with other noise draws: how big does the dead-reckoning error typically get?
    other_dr_errors = []
    for seed in range(2, 32):
        _, other_odometry, _ = AisleFrontend(sigma_odom=SIGMA_ODOM, sigma_sensor=SIGMA_SENSOR, seed=seed).run()
        other_dr_errors.append(np.abs(backend.dead_reckoning(start, other_odometry) - true_x))

    plot_aisle_results(
        frontend.posts, true_x, dr_x, est_x, est_posts, frontend.half_width,
        sigma_odom=SIGMA_ODOM, other_dr_errors=other_dr_errors, save_path="slam1d_result.png",
    )
    plt.show()


if __name__ == "__main__":
    main()
