"""Data types shared by the frontend (simulation) and the backend (estimation)."""

from dataclasses import dataclass


@dataclass
class Pose2D:
    """Where the robot is (x, y, in metres) and which way it faces (theta, in radians, 0 = along +x)."""

    x: float
    y: float
    theta: float


@dataclass
class Odometry:
    """The motion from pose `step` to pose `step + 1`, measured by the robot and expressed in
    the robot's own frame at pose `step`: dx forward, dy to the left, dtheta the turn."""

    step: int
    dx: float
    dy: float
    dtheta: float


@dataclass
class Observation:
    """One sighting of a landmark from one pose: the distance to it (range, metres) and the
    direction to it relative to the robot's heading (bearing, radians, positive = to the left).
    The robot knows which landmark it sees (`landmark_id`): data association is given."""

    pose_index: int
    landmark_id: int
    range: float
    bearing: float


@dataclass
class Observation1D:
    """The one-dimensional version, used in the single aisle: how far ahead of the robot
    the landmark is, along the aisle (negative = behind)."""

    pose_index: int
    landmark_id: int
    offset: float
