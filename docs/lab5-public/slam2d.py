"""Part 2: SLAM on the warehouse floor.

    uv run slam2d.py                run the patrol route, print the errors, show and save the plot
    uv run slam2d.py --play         the drive step by step, with whatever is implemented so far
    uv run slam2d.py --no-sparsity  solve without the sparsity pattern of the Jacobian (compare the timings)
    uv run slam2d.py --spy          also draw that pattern
"""

import argparse
import math
import time

import matplotlib.pyplot as plt
import numpy as np

from slam_core.backend_2d import WarehouseBackend, dead_reckoning
from slam_core.frontend_2d import WarehouseFrontend
from slam_core.player import play_warehouse
from slam_core.visualization import plot_jacobian_pattern, plot_warehouse_results
from slam_core.world import Warehouse

NOISE = dict(
    sigma_odom_xy=0.02,  # odometry: metres per step ...
    sigma_odom_theta=math.radians(1.0),  # ... and the turn per step
    sigma_range=0.05,  # scanner: metres ...
    sigma_bearing=math.radians(1.0),  # ... and the direction
)
SENSOR_RANGE = 4.0  # metres: the tags further away are not seen
STEP = 0.5  # metres: a pose and a set of measurements every half metre of the drive


def errors(true_poses, poses):
    truth = np.array([(p.x, p.y) for p in true_poses])
    d = np.hypot(*(poses[:, :2] - truth).T)
    return d.mean(), d.max(), d[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--play", action="store_true", help="the drive step by step, interactive")
    ap.add_argument("--no-sparsity", action="store_true", help="solve without the sparsity pattern")
    ap.add_argument("--spy", action="store_true", help="also draw the sparsity pattern of the Jacobian")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    world = Warehouse(bay=3)  # a label every 3 m along the shelves
    frontend = WarehouseFrontend(world, seed=args.seed, sensor_range=SENSOR_RANGE, step=STEP, **NOISE)
    true_poses, odometry, observations = frontend.run()
    print(world.ascii(true_poses))
    print(f"{len(true_poses)} poses, {len(observations)} observations of {len({o.landmark_id for o in observations})} tags")

    backend = WarehouseBackend(use_sparsity=not args.no_sparsity, **NOISE)
    start = world.start_pose()
    if args.play:
        play_warehouse(world, true_poses, odometry, observations, backend, STEP)
        return
    dr = dead_reckoning(start, odometry)

    t0 = time.time()
    est_poses, est_landmarks, result = backend.optimize(start, odometry, observations, verbose=0)
    print(f"solved in {time.time() - t0:.1f} s, {result.nfev} residual evaluations, final cost {2 * result.cost:.1f}")
    for name, poses in (("dead reckoning", dr), ("SLAM", est_poses)):
        mean, worst, last = errors(true_poses, poses)
        print(f"{name:15s} position error: mean {mean:.2f} m, max {worst:.2f} m, at the end {last:.2f} m")
    lm_err = [math.hypot(est_landmarks[i][0] - world.landmarks[i][0], est_landmarks[i][1] - world.landmarks[i][1]) for i in est_landmarks]
    print(f"tags: mean error {np.mean(lm_err):.2f} m, max {np.max(lm_err):.2f} m")

    plot_warehouse_results(world, true_poses, dr, est_poses, est_landmarks, save_path="slam2d_result.png")
    if args.spy:
        if result.jac is None or not hasattr(result.jac, "nnz"):
            print("the Jacobian pattern is only available with the sparsity pattern (drop --no-sparsity)")
        else:
            plot_jacobian_pattern(result.jac, len(true_poses), len(odometry), save_path="slam2d_jacobian.png")
    plt.show()


if __name__ == "__main__":
    main()
