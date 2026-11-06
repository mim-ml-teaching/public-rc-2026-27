import math

import mujoco
from mujoco import viewer
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import tyro

from drone_simulator import DroneSimulator
from pid import PID


SIM_TIME = 5000  # Maximum simulation time in steps


def xquat_to_euler(xquat):
    return R.from_quat([xquat[1], xquat[2], xquat[3], xquat[0]]).as_euler('xyz', degrees=True)


def reset_world(model, data, fixed_track: bool, rotated_gates: bool) -> None:
    mujoco.mj_resetData(model, data)
    # Mocap bodies stay fixed during flight, but can be repositioned between runs.
    for name, fixed_yaw in zip(("red_gate", "green_gate", "blue_gate"), (-15, -30, 45)):
        mocap_id = model.body(name).mocapid[0]
        if not fixed_track:
            data.mocap_pos[mocap_id, 1:] = [np.random.uniform(-0.6, 0.6), np.random.uniform(0.7, 1.3)]
        if rotated_gates:
            yaw = math.radians(fixed_yaw if fixed_track else np.random.uniform(-45, 45))
            data.mocap_quat[mocap_id] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    mujoco.mj_forward(model, data)


def run_single_task(model, data, view, *, wind: bool, rotated_gates: bool, rendering_freq: float, fixed_track: bool) -> None:
    with view.lock():
        reset_world(model, data, fixed_track, rotated_gates)
    view.sync()

    pos_targets = [
        [0, 0, 1],
        data.body("red_gate").xpos.copy().tolist(),
        data.body("green_gate").xpos.copy().tolist(),
        data.body("blue_gate").xpos.copy().tolist(),
        [-8, 0, 1]
    ]

    yaw_quat_targets = [
        [1, 0, 0, 0],
        data.body("red_gate").xquat.copy().tolist(),
        data.body("green_gate").xquat.copy().tolist(),
        data.body("blue_gate").xquat.copy().tolist(),
        [1, 0, 0, 0]
    ]

    yaw_angle_targets = [xquat_to_euler(xquat)[2] for xquat in yaw_quat_targets]

    # TODO: Design PID control
    pid_roll = PID(
        gain_prop = 0, gain_int = 0, gain_der = 0,
        sensor_period = model.opt.timestep, output_limits=(-100, 100)
    )

    pid_pitch = PID(
        gain_prop = 0, gain_int = 0, gain_der = 0,
        sensor_period = model.opt.timestep, output_limits=(-100, 100)
    )

    pid_yaw = PID(
        gain_prop = 0, gain_int = 0, gain_der = 0,
        sensor_period = model.opt.timestep, output_limits=(-100, 100)
    )
    # END OF TODO

    task_label = f"rotated={'yes' if rotated_gates else 'no'}, wind={'yes' if wind else 'no'}"
    print(f"Starting task ({task_label})")
    wind_change_prob = 0.1 if wind else 0

    # If you want the simulation to be displayed more slowly, decrease rendering_freq
    # Note that this DOES NOT change the timestep used to approximate the physics of the simulation!
    drone_simulator = DroneSimulator(
        model, data, view, wind_change_prob = wind_change_prob, rendering_freq = rendering_freq
    )

    # TODO: Define additional variables if needed
    # END OF TODO

    for _ in range(SIM_TIME):
        if not view.is_running():
            return
        current_pos, previous_pos = drone_simulator.position_sensor()
        current_orien, previous_orien = drone_simulator.orientation_sensor()

        if np.linalg.norm(np.array(current_pos) - np.array(pos_targets[-1])) < 0.2:
            break

        # TODO: define the current target position
        pos_target = pos_targets[0].copy()
        # END OF TODO

        # TODO: use PID controllers to steer the drone
        desired_thrust = 3.2496

        desired_roll = 0
        desired_pitch = 0
        desired_yaw = 0

        roll_thrust = pid_roll.output_signal(desired_roll, [current_orien[0], previous_orien[0]])
        pitch_thrust = pid_pitch.output_signal(desired_pitch, [current_orien[1], previous_orien[1]])
        yaw_thrust = -pid_yaw.output_signal(desired_yaw, [current_orien[2], previous_orien[2]])
        # END OF TODO

        # For debugging purposes you can uncomment, but keep in mind that this slows down the simulation

        # data = np.array([pos_target + [desired_roll, desired_pitch, desired_yaw], np.concat([current_pos, current_orien])]).T
        # row_names = ["x", "y", "z", "roll", "pitch", "yaw"]
        # headers = ["desired", "current"]
        # print(pd.DataFrame(data, index=row_names, columns=headers))

        drone_simulator.sim_step(
            desired_thrust, roll_thrust=roll_thrust,
            pitch_thrust=pitch_thrust, yaw_thrust=yaw_thrust
        )

    current_pos, _ = drone_simulator.position_sensor()
    assert np.linalg.norm(np.array(current_pos) - np.array(pos_targets[-1])) < 0.2, "Drone did not reach the final target!"
    print(f"Task ({task_label}) completed successfully!")


def main(
    wind: bool = False,
    rotated_gates: bool = False,
    all_tasks: bool = False,
    runs: int = 10,
    rendering_freq: float = 3.0,
    fixed_track: bool = False,
) -> None:
    """
    Run the drone control simulation.

    Args:
        wind: Enable wind disturbances.
        rotated_gates: Rotate gates to create the harder variant.
        all_tasks: Run all four combinations of wind/rotated gates.
        runs: How many times to repeat each selected task.
        rendering_freq: Viewer rendering frequency multiplier (lower slows playback).
    """
    task_list = []
    if all_tasks:
        task_list = [
            (False, False),
            (True, False),
            (False, True),
            (True, True),
        ]
    else:
        task_list = [(wind, rotated_gates)]

    model = mujoco.MjModel.from_xml_path("assets/scene.xml")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    with viewer.launch_passive(model, data) as view:
        with view.lock():
            view.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            view.cam.fixedcamid = model.camera("track").id
        for wind_flag, rotated in task_list:
            for run_idx in range(runs):
                if not view.is_running():
                    return
                print(f"\nRun {run_idx + 1}/{runs} for wind={wind_flag}, rotated_gates={rotated}")
                run_single_task(
                    model, data, view,
                    wind=wind_flag,
                    rotated_gates=rotated,
                    rendering_freq=rendering_freq,
                    fixed_track=fixed_track,
                )


if __name__ == '__main__':
    tyro.cli(main)
