"""Course assets and vectorized, swept gate checks (no Genesis dependency)."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

import torch

TRACKS_DIR = Path(__file__).resolve().parents[1] / "assets" / "tracks"
DEFAULT_TRACK = "loop"


def _vector(value, label):
    if len(value) != 3 or not all(math.isfinite(v) for v in value):
        raise ValueError(f"{label} must contain three finite numbers")
    return tuple(value)


@dataclass(frozen=True)
class Gate:
    name: str
    center: tuple[float, float, float]
    yaw_degrees: float = 0.0
    color: tuple[float, float, float] = (1.0, 0.3, 0.0)

    def __post_init__(self):
        if not self.name:
            raise ValueError("Each gate needs a name")
        object.__setattr__(self, "center", _vector(self.center, f"Gate {self.name} center"))
        object.__setattr__(self, "color", _vector(self.color, f"Gate {self.name} color"))
        if not math.isfinite(self.yaw_degrees):
            raise ValueError(f"Gate {self.name} yaw_degrees must be finite")
        if not all(0 <= channel <= 1 for channel in self.color):
            raise ValueError(f"Gate {self.name} color channels must be in [0, 1]")


@dataclass(frozen=True)
class Course:
    gates: tuple[Gate, ...]
    start: tuple[float, float, float] = (0.0, 0.0, 1.0)
    opening_width: float = 1.1
    opening_height: float = 1.2
    bar_thickness: float = 0.1
    frame_depth: float = 0.1
    name: str = "custom"
    description: str = ""
    time_limit: float = 60.0

    def __post_init__(self):
        if not self.gates:
            raise ValueError("A course needs at least one gate")
        object.__setattr__(self, "start", _vector(self.start, "Course start"))
        dimensions = (self.opening_width, self.opening_height, self.bar_thickness, self.frame_depth)
        if not all(math.isfinite(v) and v > 0 for v in dimensions):
            raise ValueError("Gate dimensions must be finite and positive")
        if not math.isfinite(self.time_limit) or self.time_limit <= 0:
            raise ValueError("time_limit must be finite and positive")
        if len({gate.name for gate in self.gates}) != len(self.gates):
            raise ValueError("Gate names must be unique within a course")

    @classmethod
    def load(cls, path: str | Path | None = None):
        path = Path(path) if path is not None else TRACKS_DIR / f"{DEFAULT_TRACK}.json"
        data = json.loads(path.read_text())
        data.pop("source", None)
        data["gates"] = tuple(Gate(**gate) for gate in data["gates"])
        return cls(**data)

    @staticmethod
    def available_tracks():
        """Discover bundled JSON configs; adding a file makes a new --track choice."""
        return tuple(sorted(path.stem for path in TRACKS_DIR.glob("*.json")))

    @classmethod
    def load_track(cls, name: str = DEFAULT_TRACK):
        if name not in cls.available_tracks():
            raise ValueError(
                f"Unknown track {name!r}; choose from {', '.join(cls.available_tracks())}"
            )
        return cls.load(TRACKS_DIR / f"{name}.json")

    def overview_camera(self):
        """Frame the whole course, including the start, without hard-coded coordinates."""
        points = [self.start, *(gate.center for gate in self.gates)]
        low = [min(p[axis] for p in points) for axis in range(3)]
        high = [max(p[axis] for p in points) for axis in range(3)]
        center = tuple((a + b) / 2 for a, b in zip(low, high))
        span = max(high[0] - low[0], high[1] - low[1], high[2] - low[2], 4.0)
        distance = 1.4 * span + 1.5 * max(self.opening_width, self.opening_height)
        position = (
            center[0] + 0.6 * distance,
            center[1] - 0.85 * distance,
            center[2] + 0.6 * distance,
        )
        return position, center

    def bars(self):
        """Local centers and full box sizes, reproducing the four lab9 gate bars."""
        w, h, t, d = (self.opening_width, self.opening_height, self.bar_thickness, self.frame_depth)
        return (
            ((0, -(w + t) / 2, 0), (d, t, h)),
            ((0, (w + t) / 2, 0), (d, t, h)),
            ((0, 0, -(h + t) / 2), (d, w + 2 * t, t)),
            ((0, 0, (h + t) / 2), (d, w + 2 * t, t)),
        )


class CourseGeometry:
    def __init__(self, course: Course, radius: float, device):
        if radius <= 0 or 2 * radius >= min(course.opening_width, course.opening_height):
            raise ValueError("Drone radius must be positive and fit inside the gates")
        self.course, self.radius = course, radius
        self.centers = torch.tensor([g.center for g in course.gates], device=device)
        yaw = torch.tensor([math.radians(g.yaw_degrees) for g in course.gates], device=device)
        self.cos, self.sin = yaw.cos(), yaw.sin()
        self.normals = torch.stack((-self.cos, -self.sin, torch.zeros_like(yaw)), dim=-1)
        self.bar_centers = torch.tensor([b[0] for b in course.bars()], device=device)
        self.bar_half = torch.tensor([b[1] for b in course.bars()], device=device) / 2

    def local(self, points):
        offset = points[:, None, :] - self.centers
        x, y, z = offset.unbind(-1)
        return torch.stack((self.cos * x + self.sin * y, -self.sin * x + self.cos * y, z), -1)

    def check(self, previous, position, next_gate):
        """Return (passed_next_gate, hit_frame, missed_next_gate), one bool per env.

        A passage crosses the plane from local +X to -X, inside the opening
        shrunk by the drone radius. Frame collision is a conservative swept
        sphere approximation: segment vs radius-expanded bar boxes. All gates
        are checked for collision, even when they are not the current target.
        """
        a, b = self.local(previous), self.local(position)
        direction = b - a
        crossing = (a[..., 0] > 0) & (b[..., 0] <= 0)
        fraction = a[..., 0] / (a[..., 0] - b[..., 0]).clamp_min(1e-12)
        intersection = a + fraction[..., None] * direction
        inside = (intersection[..., 1].abs() < self.course.opening_width / 2 - self.radius) & (
            intersection[..., 2].abs() < self.course.opening_height / 2 - self.radius
        )
        target = next_gate.clamp_max(len(self.course.gates) - 1)[:, None]
        active = next_gate < len(self.course.gates)
        passed = (crossing & inside).gather(1, target).squeeze(1) & active
        missed = (crossing & ~inside).gather(1, target).squeeze(1) & active

        # Shapes: (environments, gates, bars, xyz). Slab intersection also
        # catches a fast drone that enters and leaves a bar in a single step.
        origin = a[:, :, None, :] - self.bar_centers
        delta = direction[:, :, None, :]
        half = self.bar_half + self.radius
        parallel = delta.abs() < 1e-12
        safe_delta = torch.where(parallel, torch.ones_like(delta), delta)
        t1, t2 = (-half - origin) / safe_delta, (half - origin) / safe_delta
        lower = torch.where(parallel, -torch.inf, torch.minimum(t1, t2)).amax(-1)
        upper = torch.where(parallel, torch.inf, torch.maximum(t1, t2)).amin(-1)
        parallel_outside = (parallel & (origin.abs() > half)).any(-1)
        hit = (~parallel_outside & (lower.clamp_min(0) <= upper.clamp_max(1))).any((1, 2))
        return passed, hit, missed
