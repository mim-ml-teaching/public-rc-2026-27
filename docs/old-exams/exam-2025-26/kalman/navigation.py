import tyro
from typing import Optional
from dataclasses import dataclass

import cv2
import numpy as np
import pandas as pd

from kalman import KalmanFilter
from mouse_simulator import MouseNavigationSimulator
from replay_simulator import ReplayNavigationSimulator
from utils import floats_to_ints


def path_lengths(history: np.ndarray) -> np.ndarray:
    """Compute lenghts of K paths with points in N dimensional space.

    history: array of shape (T, K, N), where T is the number of time steps.
    """
    if len(history) < 2:
        raise ValueError("Need at least 2 history points to compute path lengths.")
    diffs = np.diff(history, axis=0)  # (T-1, K, N)
    return np.sum(np.linalg.norm(diffs, axis=2), axis=0)  # (K,)


def main(record_path: Optional[str] = None, replay_path: Optional[str] = None, headless: bool = False):
    if replay_path:
        simulator = ReplayNavigationSimulator(replay_path)
    else:
        simulator = MouseNavigationSimulator()
    
    last_gps_known_position = simulator.true_pos.copy()

    # IMU integration state
    imu_pos = simulator.true_pos.copy()
    imu_vel = np.array([0.0, 0.0])

    history = []  # Per-step positions: [Ground Truth, Kalman Filter, GPS, IMU Integrated]
    
    print("Move mouse to simulate movement.")
    print("Green: True Path (Mouse)")
    print("Blue:  Kalman Filter Estimate (Red lines are frequent IMU updates)")
    print("Red Dot: Periodic GPS Correction")
    print("Yellow: IMU Integrated Path")


    # TODO: Initialize Kalman filter
    kf = KalmanFilter(
        x=np.array([[last_gps_known_position[0]], [last_gps_known_position[1]], [0], [0]]),
    )
    # End TODO


    i = 0
    all_tests_passed = True
    try:
        while True:
            i += 1
            if not headless:
                print(f"Iteration {i}", end="\r")
        
            gps_reading, imu_reading, dt = simulator.run_step()
            
            # Exit if dt is invalid (replay ended or error)
            if dt <= 0:
                break
        
            # TODO: use Kalman filter to estimate position
            # End TODO


            ### VISUALISATION ###
            # Position estimates based on IMU integration alone
            imu_vel += imu_reading * dt
            imu_pos += imu_vel * dt + 0.5 * imu_reading * (dt ** 2)

            # Draw Estimate - Blue
            est_pos = kf.x[:2].flatten()

            if not headless:
                img = simulator.render()
                cv2.circle(img, floats_to_ints(est_pos), 2, (255, 100, 0), -1)

                # Draw IMU Integrated Position - Yellow
                cv2.circle(img, floats_to_ints(imu_pos), 2, (0, 255, 255), -1)

                # Draw GPS Measurement - Red (Big dot)
                if gps_reading is not None:
                    cv2.circle(img, floats_to_ints(gps_reading), 6, (0, 0, 255), -1)
                    cv2.circle(img, floats_to_ints(gps_reading), 8, (0, 0, 255), 1)
                    cv2.putText(img, "GPS SIGNAL", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

                cv2.imshow("IMU + GPS Navigation Simulation", img)

            if record_path:
                simulator.record_step(gps_reading, imu_reading, dt)


            ### LOGGING AND METRICS ###
            if gps_reading is not None:
                last_gps_known_position = gps_reading
            
            history.append(np.stack([
                    np.array(simulator.true_pos, copy=True),
                    np.array(est_pos, copy=True),
                    np.array(last_gps_known_position, copy=True),
                    np.array(imu_pos, copy=True),
                ]))
            if len(history) > 500:
                history.pop(0)

            if i % 500 == 0:
                p_lengths = path_lengths(np.array(history))
                
                # Calculate errors
                kf_path_error = abs(p_lengths[1] - p_lengths[0])
                gps_path_error = abs(p_lengths[2] - p_lengths[0])
                kf_pos_error = float(np.linalg.norm(simulator.true_pos - est_pos))
                imu_pos_error = float(np.linalg.norm(simulator.true_pos - imu_pos))
                
                # Test criteria
                checkpoint_num = i // 500
                
                # Test 1: KF path length error should be reasonable
                if p_lengths[0] < 4000:
                    gps_division_factor = 5.0
                elif p_lengths[0] < 7000:
                    gps_division_factor = 25.0
                else:
                    gps_division_factor = 43.0
                test1_passed = kf_path_error <= 50.0 or kf_path_error < gps_path_error / gps_division_factor
                
                # Test 2: KF position error should be below absolute threshold
                test2_passed = kf_pos_error < 40.0
                
                overall_passed = test1_passed and test2_passed
                
                # Only print output if not in headless mode or if a test failed
                if not headless or not overall_passed:
                    print("\n--- Metrics after {} iterations ---".format(i))
                    metrics = pd.DataFrame({
                        "Ground Truth": [p_lengths[0], 0.0, f"({simulator.true_pos[0]:.2f}, {simulator.true_pos[1]:.2f})", 0.0],
                        "Kalman Filter": [p_lengths[1], p_lengths[1] - p_lengths[0], f"({est_pos[0]:.2f}, {est_pos[1]:.2f})", kf_pos_error],
                        "GPS": [p_lengths[2], p_lengths[2] - p_lengths[0], f"({last_gps_known_position[0]:.2f}, {last_gps_known_position[1]:.2f})", float(np.linalg.norm(simulator.true_pos - last_gps_known_position))],
                        "IMU": [p_lengths[3], p_lengths[3] - p_lengths[0], f"({imu_pos[0]:.2f}, {imu_pos[1]:.2f})", imu_pos_error],
                    }, index=["Path lengths", "Error for path length", "Current positions", "Error for current positions"])
                    print(metrics)
                    
                    print("\n--- Test Results (Checkpoint {}) ---".format(checkpoint_num))
                    
                    status1 = f"\033[92mPASS\033[0m" if test1_passed else f"\033[91mFAIL\033[0m"
                    print(f"Test 1 - Path Length Error: KF={kf_path_error:.2f} vs GPS/{gps_division_factor:.0f}={gps_path_error/gps_division_factor:.2f} ... {status1}")
                    
                    status2 = f"\033[92mPASS\033[0m" if test2_passed else f"\033[91mFAIL\033[0m"
                    print(f"Test 2 - Position Error Threshold: KF={kf_pos_error:.2f} vs Threshold=40.00 ... {status2}")
                    
                    overall_status = f"\033[92mPASS\033[0m" if overall_passed else f"\033[91mFAIL\033[0m"
                    print(f"\nOverall Status: {overall_status}")
                    print("-" * 50)
                
                # Track if any test failed
                if not overall_passed:
                    all_tests_passed = False
            

            if not headless:
                if cv2.waitKey(20) == 27: # ESC
                    break
    finally:
        if record_path:
            simulator.save_recording(record_path)
        
        # Print final status if all tests passed and we completed a replay
        if all_tests_passed and replay_path:
            print(f"\033[92mAll tests passed.\033[0m")
        elif not all_tests_passed and replay_path:
            print(f"\033[91mSome tests failed. Review the output above.\033[0m")


@dataclass
class Args:
    record: Optional[str] = None
    replay: Optional[str] = None
    headless: bool = False


if __name__ == "__main__":
    args = tyro.cli(Args)
    main(record_path=args.record, replay_path=args.replay, headless=args.headless)
