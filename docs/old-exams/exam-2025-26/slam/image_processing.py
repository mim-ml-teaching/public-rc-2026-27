from typing import Any, Dict, List, Tuple

import cv2
import numpy as np

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)


def check_is_tag_good(image: np.ndarray, corners: np.ndarray) -> bool:
    """
    Checks if the detected tag is a "good" tag by analyzing its border.

    Args:
        image: RGB image (numpy array).
        corners: np.ndarray of shape (4, 2), the 2D pixel coordinates of the 4 corners.

    Returns:
        True if the tag is good, False if it is bad.
    """
    is_tag_good = True
    ### TODO: If the tag is "bad" (has red border) return False, otherwise return True.

    ### END TODO

    return is_tag_good


def find_aruco_tags(image: np.ndarray, check_tag: bool = True) -> List[Dict[str, Any]]:
    """
    Detects ArUco tags in the provided image.

    Args:
        image: RGB image (numpy array).
        check_tag: If True, filter out "bad" tags using check_is_tag_good, if False return all detected tags, even "bad" ones.
        IMPORTANT: this flag WILL be used during automated testing, make sure to implement it.

    Returns:
        List of dictionaries. Each dictionary contains:
            - 'id': int, the ID of the tag.
            - 'corners': np.ndarray of shape (4, 2), the 2D pixel coordinates of the 4 corners.
    """
    detected_tags = []
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    ### TODO: Detect Aruco tags in the image. Return a list of dictionaries with 'id' and 'corners' for each tag.

    ### END TODO

    return detected_tags


def draw_aruco_tags(
    image: np.ndarray, detected_tags: List[Dict[str, Any]]
) -> np.ndarray:
    ### TODO: For each detected tag, draw its bounding box and ID on the image.
    ### NOTE: This is not mandatory, and no points will be given for this part, but it might help you debug your tag detection.

    ### END TODO
    return image


def estimate_tags_poses(
    detected_tags: List[Dict[str, Any]],
    tag_sizes: Dict[int, float],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Dict[int, Dict[str, np.ndarray]]:
    """
    Estimates the precision 3D pose of each tag relative to the camera.

    Args:
        detected_tags: List of tag dicts (from find_aruco_tags).
        tag_sizes: Dict mapping tag ID to physical size in meters.
        camera_matrix: Intrinsic camera matrix.
        dist_coeffs: Distortion coefficients.

    Returns:
        Dict mapping tag ID to a dictionary containing:
            - 'rvec': Rotation vector (3x1).
            - 'tvec': Translation vector (3x1).
    """
    tag_poses = {}

    ### TODO: For each detected tag, estimate its pose relative to the camera using solvePnP.
    ### tag poses should contain a key for each detected tag id, with 'rvec' and 'tvec' as values.

    ### END TODO

    return tag_poses


def process_pose(
    tvec: np.ndarray, rvec: np.ndarray, cube_size: float
) -> Tuple[float, float]:
    """
    Calculates the 2D ground-plane distance and angle to the cube's center.

    Args:
        tvec: Translation vector from camera to tag.
        rvec: Rotation vector of the tag.
        cube_size: Size of the tag/cube in meters.

    Returns:
        dist: Distance in the X-Z plane of the camera frame.
        angle: Angle in the X-Z plane (rotation around Y axis).
    """

    ### TODO: Given tvec and rvec of a tag, and the size of the cube compute the distance and angle from camera to the center of the CUBE.
    ### NOTE: Even though our car is in 3d we only care about the distance in the GROUND PLANE (x,y in mujoco) and the angle around the z axis (in mujoco).
    ### Remeber that axes in camera frame are: x - right, y - down, z - forward, which is different from mujoco.

    ### END TODO

    return dist, angle


def process_all_images(images, tag_sizes, camera_matrix, dist_coeffs):
    # We are grouping measurements by tag id
    per_tag_measurements = {}

    for i, img in enumerate(images):
        # First we detect tags
        detected_tags = find_aruco_tags(img)
        # Then we estimate poses of tags relative to camera
        tag_poses = estimate_tags_poses(
            detected_tags, tag_sizes, camera_matrix, dist_coeffs
        )

        for id, pose in tag_poses.items():
            # We find relative distance and angle of the landmark
            landmark_pose = process_pose(pose["tvec"], pose["rvec"], tag_sizes[id])
            if id not in per_tag_measurements:
                per_tag_measurements[id] = []

            per_tag_measurements[id].append({
                "pose_idx": i,
                "distance_angle": landmark_pose,
            })

    return per_tag_measurements
