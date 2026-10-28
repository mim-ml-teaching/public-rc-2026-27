"""The warehouse: a grid of shelves (walls) and aisles (free cells), drawn as ASCII art.

Every cell is 1 m x 1 m. '#' is a shelf, '.' is floor, 'S' is where the robot starts (floor).
The landmarks are tags on the shelves: one at every corner of a shelf block and a label every
few metres along the shelf faces. The robot drives down the middle of the aisles, rounds the
corners, and logs a pose at every `step` metres of its drive.
"""

import math
from collections import deque
from typing import Dict, List, Optional, Sequence, Tuple

from .models import Pose2D

Cell = Tuple[int, int]  # (column, row) with row 0 at the bottom

DEFAULT_MAP = """
#####################
#S..........#.......#
#.#######.#.#.#####.#
#.#.......#.#.....#.#
#.#.#########.###.#.#
#.#...........#...#.#
#.#######.#####.###.#
#.......#.......#...#
#######.#.#####.#.###
#.......#...#...#...#
#.###########.#####.#
#...................#
#####################
"""

# The patrol route: the robot visits these cells in order ((column, row), row 0 at the bottom) and ends at the start.
# The path between consecutive waypoints is the shortest one through the aisles.
DEFAULT_WAYPOINTS = [(13, 11), (19, 1), (1, 1), (1, 11)]


class Warehouse:
    def __init__(self, ascii_map: str = DEFAULT_MAP, bay: int = 2):
        self.bay = bay  # a label every `bay` metres along the shelf faces
        rows = [line for line in ascii_map.strip("\n").split("\n")]
        self.height = len(rows)
        self.width = max(len(r) for r in rows)
        self.walls = set()
        self.start_cell: Optional[Cell] = None
        for r, line in enumerate(rows):
            row = self.height - 1 - r  # row 0 at the bottom
            for c, ch in enumerate(line):
                if ch == "#":
                    self.walls.add((c, row))
                elif ch == "S":
                    self.start_cell = (c, row)
        if self.start_cell is None:
            raise ValueError("the map needs a start cell 'S'")
        self.landmarks: Dict[int, Tuple[float, float]] = self._place_landmarks()

    # ---- geometry -------------------------------------------------------------------------------
    def is_free(self, cell: Cell) -> bool:
        c, r = cell
        return 0 <= c < self.width and 0 <= r < self.height and cell not in self.walls

    @staticmethod
    def cell_centre(cell: Cell) -> Tuple[float, float]:
        return cell[0] + 0.5, cell[1] + 0.5

    def start_pose(self) -> Pose2D:
        x, y = self.cell_centre(self.start_cell)
        return Pose2D(x, y, 0.0)

    def _place_landmarks(self) -> Dict[int, Tuple[float, float]]:
        """The tags on the shelves: one at every corner of a shelf block (a grid vertex where the
        2x2 cells around it contain exactly one shelf cell, an outer corner, or exactly three, an
        inner corner), and one in the middle of every other shelf face along the aisles (a label
        on every bay, `self.bay` metres apart)."""
        tags = []
        for vx in range(self.width + 1):
            for vy in range(self.height + 1):
                around = [(vx - 1, vy - 1), (vx, vy - 1), (vx - 1, vy), (vx, vy)]
                n_walls = sum(1 for cell in around if cell in self.walls)
                n_floor = sum(1 for cell in around if self.is_free(cell))
                if n_floor >= 1 and n_walls in (1, 3):
                    tags.append((float(vx), float(vy)))
        for c, r in sorted(self.walls):
            if self.is_free((c, r + 1)) and c % self.bay == 0:
                tags.append((c + 0.5, r + 1.0))  # the top face of this shelf cell
            if self.is_free((c, r - 1)) and c % self.bay == 0:
                tags.append((c + 0.5, float(r)))  # the bottom face
            if self.is_free((c + 1, r)) and r % self.bay == 0:
                tags.append((c + 1.0, r + 0.5))  # the right face
            if self.is_free((c - 1, r)) and r % self.bay == 0:
                tags.append((float(c), r + 0.5))  # the left face
        return {i: p for i, p in enumerate(sorted(set(tags)))}

    def blocked(self, x: float, y: float, margin: float = 0.03) -> bool:
        """Is the point strictly inside a shelf (not just on its edge)?"""
        c, r = math.floor(x), math.floor(y)
        if (c, r) not in self.walls:
            return False
        fx, fy = x - c, y - r
        return margin < fx < 1 - margin and margin < fy < 1 - margin

    def line_of_sight(self, p: Tuple[float, float], q: Tuple[float, float]) -> bool:
        """Can the robot at p see the tag at q, i.e. does the segment p-q avoid the shelves?"""
        dist = math.hypot(q[0] - p[0], q[1] - p[1])
        n = max(2, int(dist / 0.02))
        for k in range(n):  # the last sample (the tag itself, on a shelf corner) is skipped
            t = k / n
            if self.blocked(p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])):
                return False
        return True

    # ---- routes -----------------------------------------------------------------------------------
    def shortest_path(self, a: Cell, b: Cell) -> List[Cell]:
        """Breadth-first search through the aisles."""
        prev = {a: None}
        queue = deque([a])
        while queue:
            cur = queue.popleft()
            if cur == b:
                break
            for d in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (cur[0] + d[0], cur[1] + d[1])
                if self.is_free(nxt) and nxt not in prev:
                    prev[nxt] = cur
                    queue.append(nxt)
        if b not in prev:
            raise ValueError(f"no path from {a} to {b}")
        path = [b]
        while path[-1] != a:
            path.append(prev[path[-1]])
        return path[::-1]

    def route(self, waypoints: Sequence[Cell] = DEFAULT_WAYPOINTS) -> List[Tuple[float, float]]:
        """The centre line of the drive: the centres of the cells from the start through the
        waypoints, along the shortest path between consecutive ones."""
        cells = [self.start_cell]
        for w in waypoints:
            if not self.is_free(w):
                raise ValueError(f"waypoint {w} is not on the floor")
            cells += self.shortest_path(cells[-1], w)[1:]
        return [self.cell_centre(c) for c in cells]

    def drive(self, waypoints: Sequence[Cell] = DEFAULT_WAYPOINTS, step: float = 0.5, turn_radius: float = 0.4) -> List[Pose2D]:
        """Drive the route: straight down the aisles, around the corners on an arc of `turn_radius`,
        and log the pose (position and heading, the direction of travel) every `step` metres."""
        corners = self.route(waypoints)
        # the drive as a list of pieces: straight segments and arcs between them
        pieces = []  # each: (length, function s -> (x, y, theta)) for s in [0, length]
        heading = lambda p, q: math.atan2(q[1] - p[1], q[0] - p[0])
        # the straight pieces are cut short by turn_radius at every corner that has a turn
        points = [corners[0]]
        for i in range(1, len(corners) - 1):
            a, b, c = corners[i - 1], corners[i], corners[i + 1]
            th_in, th_out = heading(a, b), heading(b, c)
            if abs(wrap_angle(th_out - th_in)) < 1e-9:
                continue  # straight on: no corner here
            points.append((b, th_in, th_out))
        points.append(corners[-1])

        cursor = corners[0]
        for item in points[1:]:
            if isinstance(item[0], tuple):  # a corner: straight piece up to it, then the arc
                b, th_in, th_out = item
                end = (b[0] - turn_radius * math.cos(th_in), b[1] - turn_radius * math.sin(th_in))
                pieces.append(_straight(cursor, end, th_in))
                turn = wrap_angle(th_out - th_in)  # +90 or -90 degrees
                centre = (end[0] - math.sin(th_in) * turn_radius * (1 if turn > 0 else -1), end[1] + math.cos(th_in) * turn_radius * (1 if turn > 0 else -1))
                pieces.append(_arc(centre, turn_radius, th_in, turn))
                cursor = (b[0] + turn_radius * math.cos(th_out), b[1] + turn_radius * math.sin(th_out))
            else:  # the final straight piece
                pieces.append(_straight(cursor, item, heading(cursor, item)))
        total = sum(L for L, _ in pieces)
        poses = []
        s = 0.0
        while s <= total + 1e-9:
            t = s
            for L, f in pieces:
                if t <= L + 1e-9:
                    x, y, th = f(min(t, L))
                    poses.append(Pose2D(x, y, th))
                    break
                t -= L
            s += step
        for p in poses:
            if not self.is_free((math.floor(p.x), math.floor(p.y))):
                raise ValueError("the route drives into a shelf")
        return poses

    def ascii(self, poses: Optional[Sequence[Pose2D]] = None) -> str:
        """The map as text, with the drive overlaid (digits: tens of the step index at which a cell was visited)."""
        grid = [["#" if (c, r) in self.walls else "." for c in range(self.width)] for r in range(self.height)]
        if poses:
            for i, p in enumerate(poses):
                c, r = math.floor(p.x), math.floor(p.y)
                if grid[r][c] == ".":
                    grid[r][c] = str((i // 10) % 10)
        c, r = self.start_cell
        grid[r][c] = "S"
        return "\n".join("".join(row) for row in grid[::-1])


def _straight(p, q, theta):
    L = math.hypot(q[0] - p[0], q[1] - p[1])
    return L, (lambda s: (p[0] + s * math.cos(theta), p[1] + s * math.sin(theta), theta))


def _arc(centre, radius, theta_in, turn):
    """A circular arc of `radius` that starts heading `theta_in` and turns by `turn` radians."""
    L = radius * abs(turn)
    sign = 1 if turn > 0 else -1
    phi0 = theta_in - sign * math.pi / 2  # the angle of the start point as seen from the centre

    def f(s):
        phi = phi0 + sign * s / radius
        return centre[0] + radius * math.cos(phi), centre[1] + radius * math.sin(phi), wrap_angle(theta_in + sign * s / radius)

    return L, f


def wrap_angle(a: float) -> float:
    """Bring an angle into (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))
