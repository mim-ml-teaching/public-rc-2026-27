"""The motion model and the sensor model of the robot: the f and h of the lecture. They are used
twice: by the frontend to simulate what the robot measures, and by the backend to predict what
it should measure for a given estimate. They work on plain numbers and on numpy arrays alike."""

import numpy as np


def wrap_angle(a):
    """Bring an angle (or an array of angles) into (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


def compose(x, y, theta, dx, dy, dtheta):
    """Dead reckoning step: the robot at (x, y, theta) moves by (dx, dy, dtheta) measured in its
    own frame (dx forward, dy to the left). Returns the new (x, y, theta). The inverse of
    `relative_motion` below."""
    ### TODO ###
    # rotate the motion from the robot's frame into the world frame, add it, wrap the angle
    raise NotImplementedError  # your code here
    ### END TODO ###


def relative_motion(xa, ya, tha, xb, yb, thb):
    """The motion model, the inverse of `compose`: the motion (dx, dy, dtheta) that takes pose a
    to pose b, expressed in the frame of pose a. This is what the odometry measures."""
    gx, gy = xb - xa, yb - ya  # the displacement in the world frame
    dx = np.cos(tha) * gx + np.sin(tha) * gy  # rotated into the frame of pose a
    dy = -np.sin(tha) * gx + np.cos(tha) * gy
    dtheta = wrap_angle(thb - tha)
    return dx, dy, dtheta


def observe(x, y, theta, lx, ly):
    """The sensor model: the range and the bearing of the landmark at (lx, ly) as seen from the
    robot at (x, y, theta). The bearing is relative to the robot's heading, positive to the left."""
    r = np.hypot(lx - x, ly - y)
    bearing = wrap_angle(np.arctan2(ly - y, lx - x) - theta)
    return r, bearing


def landmark_from_observation(x, y, theta, r, bearing):
    """The inverse of `observe`: where the landmark is if the robot at (x, y, theta) sees it at
    (r, bearing). Used only for the initial guess."""
    return x + r * np.cos(theta + bearing), y + r * np.sin(theta + bearing)
