import mujoco
import numpy as np
from mujoco import viewer
from typing import List
import time
import math
import xml.etree.ElementTree as ET

NUM_BAD_TAGS = 3


def extract_yaw_from_quaternion(quat: np.ndarray) -> float:
    """Extracts yaw angle from a quaternion."""
    base_vec = np.array([1.0, 0.0, 0.0])
    rotated_vec = np.empty(3, np.float64)
    mujoco.mju_rotVecQuat(rotated_vec, base_vec, quat)
    yaw = np.arctan2(rotated_vec[1], rotated_vec[0])
    return yaw


def add_tag(
    worldbody,
    assets,
    tag_path: str,
    tag_location: tuple,
    tag_size: float,
    tag_id: int,
    is_bad: bool,
):
    bad_suffix = "_bad" if is_bad else ""
    ET.SubElement(
        assets,
        "texture",
        {
            "name": f"tag_tex_{tag_id}{bad_suffix}",
            "type": "cube",
            "file": tag_path,
        },
    )
    ET.SubElement(
        assets,
        "material",
        {
            "name": f"tag_mat_{tag_id}{bad_suffix}",
            "texture": f"tag_tex_{tag_id}{bad_suffix}",
            "reflectance": "0.0",
            "emission": "1.0",
        },
    )
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": f"tag_{tag_id}{bad_suffix}",
            "type": "box",
            "size": f"{tag_size / 2} {tag_size / 2} {tag_size / 2}",
            "material": f"tag_mat_{tag_id}{bad_suffix}",
            "pos": f"{tag_location[0]} {tag_location[1]} {tag_location[2]}",
            "contype": "0",
            "conaffinity": "0",
        },
    )


class SLAMMujocoFrontend:
    def __init__(
        self,
        model_path: str,
        tags: List,
        num_steps_per_sensors: int = 10,
        odometry_noise_std: float = 0.0,
        initial_guess_noise_std: float = 0.0,
        visualize: bool = False,
    ):
        tree = ET.parse(model_path)
        root = tree.getroot()
        worldbody = root.find("worldbody")
        assets = root.find("asset")

        self.true_landmarks: List[np.ndarray] = []
        self.tag_sizes = {}

        for i, (tag_path, tag_location, tag_size, tag_id) in enumerate(tags):
            add_tag(
                worldbody,
                assets,
                tag_path,
                tag_location,
                tag_size,
                tag_id,
                is_bad=False,
            )
            self.tag_sizes[tag_id] = tag_size

            if "bad" not in tag_path:
                self.true_landmarks.append(np.array([tag_location[0], tag_location[1]]))

        xml_string = ET.tostring(root, encoding="unicode")
        self.model = mujoco.MjModel.from_xml_string(xml_string)
        self.data = mujoco.MjData(self.model)
        self.visualize = visualize
        if visualize:
            self.viewer = viewer.launch_passive(self.model, self.data)
        self.timestep = 0
        self.num_steps_per_sensors = num_steps_per_sensors
        self.odometry_noise_std = odometry_noise_std
        self.initial_guess_noise_std = initial_guess_noise_std

        self.true_poses: List[np.ndarray] = []

        self.true_odometry_measurements: List[np.ndarray] = []
        self.noisy_odometry_measurements: List[np.ndarray] = []
        self.camera_measurements: List[np.ndarray] = []
        self.renderer = mujoco.Renderer(self.model, height=480, width=640)

        self.last_pose = self.extract_pose(self.data.qpos)

    def extract_pose(self, qpos: np.ndarray) -> np.ndarray:
        x = qpos[0]
        y = qpos[1]
        yaw = extract_yaw_from_quaternion(qpos[3:7])
        return np.array([x, y, yaw])

    def record_measurements(self) -> None:
        prev_angle = self.last_pose[2]

        pose = self.extract_pose(self.data.qpos)

        self.true_poses.append(pose)

        delta = pose - self.last_pose
        self.last_pose = pose

        delta_x = delta[0]
        delta_y = delta[1]
        delta_theta = delta[2]

        cos_t = math.cos(-prev_angle)
        sin_t = math.sin(-prev_angle)
        rotated_dx = cos_t * delta_x - sin_t * delta_y
        rotated_dy = sin_t * delta_x + cos_t * delta_y
        delta_x, delta_y = rotated_dx, rotated_dy

        delta = np.array([delta_x, delta_y, delta_theta])

        self.true_odometry_measurements.append(delta)
        self.noisy_odometry_measurements.append(
            delta + np.random.normal(0, self.odometry_noise_std, size=3)
        )

        self.renderer.update_scene(data=self.data, camera="dash cam")
        self.camera_measurements.append(self.renderer.render())

    def step(self, controls) -> None:
        step_start = time.time()

        for control_name, control_value in controls.items():
            self.data.actuator(control_name).ctrl = control_value

        mujoco.mj_step(self.model, self.data)

        if self.timestep % self.num_steps_per_sensors == 0:
            self.record_measurements()

        if self.visualize:
            self.viewer.sync()
            time_until_next_step = self.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
        self.timestep += 1

    def get_measurements(self):
        return self.noisy_odometry_measurements, self.camera_measurements

    def get_camera_parameters(self):
        cam = self.model.camera("dash cam")
        fx = (
            0.5 * self.renderer.height / math.tan(cam.fovy.flatten()[0] * math.pi / 360)
        )
        fy = fx
        cx = self.renderer.width / 2
        cy = self.renderer.height / 2

        camera_matrix = np.array(
            [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float32
        )

        dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        return camera_matrix, dist_coeffs

    def get_initial_guess(self):
        initial_poses_guess = [
            pose + np.random.normal(0, self.initial_guess_noise_std, size=3)
            for pose in self.true_poses
        ]
        initial_landmarks_guess = [
            landmark + np.random.normal(0, self.initial_guess_noise_std, size=2)
            for landmark in self.true_landmarks
        ]
        return initial_poses_guess, initial_landmarks_guess


if __name__ == "__main__":
    tags = [
        ("assets/tags/tag_0.png", (2, 2, 0.2)),
        ("assets/tags/tag_1.png", (1, -1, 0.2)),
        ("assets/tags/tag_2.png", (2, 0, 0.2)),
    ]
    frontend = SLAMMujocoFrontend(
        model_path="assets/slam.xml",
        tags=tags,
        num_steps_per_sensors=10,
        visualize=True,
    )

    num_total_steps = 10000
    for step in range(num_total_steps):
        controls = {
            "forward": 0.1,
            "turn": 0.5,
        }
        frontend.step(controls)

    print("True Odometry Measurements:")
    for measurement in frontend.true_odometry_measurements:
        print(measurement)

    print(f"Number of camera measurements: {len(frontend.camera_measurements)}")
