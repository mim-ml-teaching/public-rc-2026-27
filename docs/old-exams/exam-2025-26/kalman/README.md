---
title: Robot Control - Kalman Filter Navigation - 2025 / 26
---

# Updates:
None yet

# Submission format

**You should submit via moodle before the deadline.**

Before submitting, go through the list below and make sure you took care of all of the requirements.
More details can be found in the detailed task description.
**If you do not comply with these regulations you can be penalized up to obtaining zero points for the task**.

1. Submit a zipped file with only the two following files inside:
   - `kalman.py`
   - `navigation.py`
2. You should modify only the places in the code which have the `TODO` tags.
3. The simulation should not crash at any stage.
4. Do not modify simulator files (`mouse_simulator.py`, `replay_simulator.py`, `navigation_simulator.py`, `utils.py`) - they will not be part of your submission.
5. The first two elements of your Kalman Filter state vector `x` must be position `[px, py, ...]` (more details below).
6. In `navigation.py`, you are allowed to modify/add only:
    - the Kalman filter initialization, and
    - the calls to the predict and update methods.

    Any other changes in `navigation.py` are strictly forbidden.
This ensures that the correctness of your solution is due to a properly implemented Kalman filter,
and not due to ad-hoc fixes, heuristics, or simulator-specific tricks.

# Problem Description

## Overview

You will implement a Kalman Filter to fuse IMU (accelerometer) and GPS measurements for 2D position tracking.
This simulates real-world navigation scenarios such as a smartphone tracking your position while running or driving.

The task simulates a navigation scenario where:
- **IMU** provides frequent (every timestep) acceleration measurements.
While each measurement is relatively accurate, errors accumulate through integration,
causing position estimates to drift over time.
- **GPS** provides infrequent position measurements.
Each measurement is less precise but provides absolute position without drift.

Your Kalman Filter must effectively combine these complementary sensors:
high-frequency IMU data for smooth tracking, corrected by absolute GPS measurements to prevent drift.
The result should outperform using either sensor alone.

An example video recorded with a working solution is provided in the repository (`example-solution.mp4`).
You can notice how the Kalman filtered estimates (blue) stay near the ground truth (green),
while the estimates based on raw IMU measurements (yellow) drift away. 

## Grading

#### Total: 16 points

Your solution will be evaluated on a test trajectory:
- two public test trajectories are provided in the repository (`tests_public*.json`)
- a secret test trajectory will be used for final grading

Points are awarded based on the number of tests passed.
Partial credit is given, but solutions that fail consistently will receive minimal or zero points.

To get the full grade, your Kalman Filter must satisfy **both** of the following criteria at every checkpoint (every 500 iterations):

#### Test 1 - Path Length Error:

KF path length error must be:
- **5x smaller** than GPS path error for paths shorter than 4000 pixels
- **50x smaller** than GPS path error for paths longer or equal than 4000 pixels

#### Test 2 - Position Error:
- KF position error must be **< 40 pixels** at all times

## Implementation Details

### Kalman Filter Design

You must implement three methods in `kalman.py`:
1. `__init__` - Initialize the filter state and parameters
2. `predict` - Prediction step using system dynamics
3. `update` - Update step using GPS measurements

**State Vector:** The first two elements of the state vector `x` **must be position** `[px, py, ...]`.
You may extend the state with velocity, or other variables as needed.

**Dynamics Modeling:** You have freedom to choose your dynamics model. Consider:
- Velocity and acceleration can be part of the state and/or treated as control inputs (i.e., $Ax + Bu$ model)
- Constant velocity, constant acceleration, or other motion models are all acceptable
- **Any design that passes the tests is valid**

**Sensor Noise:** Sensor errors are determined by `simulator.imu_noise_std` and `simulator.gps_noise_std`.

### Navigation Script

In `navigation.py`, implement the Kalman Filter integration in the `TODO` sections:
1. Initialize the Kalman Filter with appropriate state and parameters
2. Call `predict()` with acceleration measurements and timestep `dt`
3. Call `update()` with GPS measurements (when available)

### Running the Simulation

**Interactive mode (mouse control):**
```bash
uv run navigation.py
```
Move your mouse to simulate movement. Press ESC to exit.

**Replay recorded trajectories:**
```bash
uv run navigation.py --replay tests_public.json
```
Shows visualization and prints test results at each checkpoint.

**Record additional trajectories:**
```bash
uv run navigation.py --record my_test.json
```
Your mouse movements and sensor readings will be saved.

**Headless testing (fast, no visualization):**
```bash
uv run navigation.py --replay tests_public.json --headless
```
Only prints output if tests fail. Shows "All tests passed." in green if successful.

### Sensor Information

- **IMU (Accelerometer):** Returns 2D acceleration `[ax, ay]` at every timestep
- **GPS:** Returns 2D position `[px, py]` or `None` if unavailable

### Visualization

When running with visualization:
- **Green trail**: Ground truth (mouse/replay)
- **Blue dots**: Kalman Filter estimate
- **Red dot**: GPS measurement (when received)
- **Yellow dots**: Pure IMU integration (shows drift)

### Tips and Hints

1. A working solution can be implemented in fewer than 70 lines of code.
Do not overcomplicate the implementation.
Focus on clean structure and good design instead.
2. When designing your model, think in terms of a concrete real-life situation.
For example, when driving a car you control (at least partially) certain variables (what do you control with the gas pedal?),
while others simply emerge as part of the system state.
Having such perspective usually leads to models that are easier to understand and reason about.
At the same time, remember that you have flexibility in choosing the model,
and that good results can be achieved with multiple reasonable — and to some extent simplified — modeling choices.
3. Formula for the uniformly accelerated rectilinear ($a=const$) motion $x(t) = x_0 + v_0 t + \frac{at^2}{2}$
4. Think carefully when designing the noise matrices.
For example, in class we used a constant-velocity model, `x_next = x_current + v * dt`.
Imagine you have such a model and some velocity sensor.
How does error in the velocity measurement propagate into position error under this update?
How does that depend on time?
By asking and answering questions like these for your current model — i.e.,
tracking how uncertainty in each measured or assumed quantity flows through the dynamics —
you can choose Q and R more systematically and often get better results than with random trial-and-error.
5. Test with `--headless` flag for fast iteration on `tests_public.json`

Good luck!
