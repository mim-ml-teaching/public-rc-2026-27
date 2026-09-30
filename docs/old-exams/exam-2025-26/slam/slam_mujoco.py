import os
import matplotlib.pyplot as plt
from frontend_mujoco import SLAMMujocoFrontend
from backend_mujoco import optimize_slam
from visualization import plot_results_2d
from image_processing import find_aruco_tags, draw_aruco_tags, process_all_images
import tyro
import json
import time

ODOMETRY_NOISE_STD = 0.1
INITIAL_GUESS_NOISE_STD = 0.2
NUM_STEPS_PER_SENSORS = 300


def main(
    trajectory: tyro.conf.Positional[str],
    visualize: bool = False,
    result_save_path: str | None = None,
):
    """
    Main function for SLAM in mujoco environment.

    Args:
        visualize: Turn on mujoco passive viewer to see robots movement.
        save_debug_images: Save camera outputs and tag detections to 'debug' folder.
    """

    with open(trajectory, "r") as f:
        trajectory_data = json.load(f)

    tags = trajectory_data["tags"]
    controls = trajectory_data["controls"]
    # Initialize Frontend
    frontend = SLAMMujocoFrontend(
        model_path="assets/slam.xml",
        tags=tags,
        odometry_noise_std=ODOMETRY_NOISE_STD,
        visualize=visualize,
        initial_guess_noise_std=INITIAL_GUESS_NOISE_STD,
        num_steps_per_sensors=NUM_STEPS_PER_SENSORS,
    )

    # We wait a bit before starting the simulation, to give time to change/rotate the camera if needed
    if visualize:
        time.sleep(5.0)

    for c in controls:
        frontend.step(c)

    if visualize:
        frontend.viewer.close()

    odometry_measurements, camera_measurements = frontend.get_measurements()

    # For easier debugging we save each image captured by camera and draw the detected tags
    os.makedirs("debug", exist_ok=True)
    for i, img in enumerate(camera_measurements):
        plt.imsave(f"debug/{i}.png", img)
        detected_unfiltered_tags = find_aruco_tags(img, check_tag=False)
        image_with_unfiltered_tags = draw_aruco_tags(
            img.copy(), detected_unfiltered_tags
        )
        plt.imsave(f"debug/{i}_tags_unfiltered.png", image_with_unfiltered_tags)

        detected_filtered_tags = find_aruco_tags(img, check_tag=True)
        image_with_filtered_tags = draw_aruco_tags(img.copy(), detected_filtered_tags)
        plt.imsave(f"debug/{i}_tags_filtered.png", image_with_filtered_tags)

    camera_matrix, dist_coeffs = frontend.get_camera_parameters()
    detections = process_all_images(
        camera_measurements, frontend.tag_sizes, camera_matrix, dist_coeffs
    )

    initial_poses_guess, initial_landmarks_guess = frontend.get_initial_guess()

    optimized = optimize_slam(
        initial_poses_guess,
        initial_landmarks_guess,
        odometry_measurements,
        detections,
        ((0.0, 0.0), 0.0),
        ODOMETRY_NOISE_STD,
    )

    num_poses = len(initial_poses_guess) * 3
    optimized_poses = optimized.x[:num_poses]
    optimized_landmarks_flat = optimized.x[num_poses:]
    optimized_landmarks = []
    for i in range(0, len(optimized_landmarks_flat), 2):
        optimized_landmarks.append((
            optimized_landmarks_flat[i],
            optimized_landmarks_flat[i + 1],
        ))

    plot_results_2d(
        true_poses=frontend.true_poses,
        true_landmarks=frontend.true_landmarks,
        odometry_measurements=odometry_measurements,
        optimized_poses=optimized_poses,
        optimized_landmarks=optimized_landmarks,
        initial_pose=((0.0, 0.0), 0.0),
        save_path=result_save_path
        if result_save_path is not None
        else "slam_result.png",
    )


if __name__ == "__main__":
    tyro.cli(main)
