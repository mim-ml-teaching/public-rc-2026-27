"""An interactive player: the drive step by step, with whatever is implemented so far.

Keys: space = play / pause, right and left arrows = one step, up and down = faster / slower,
Home and End = the start and the end of the drive. The true drive and the sightings are always
shown; dead reckoning appears once `compose` works; SLAM appears once the residuals work. The
estimate at step t uses the measurements up to t only, warm-started from the estimate at t-1,
which is exactly what an online SLAM system does.
"""

from typing import Callable, Optional

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection

from .geometry import compose, landmark_from_observation
from .visualization import BLUE, GREEN, GREY, RED, draw_warehouse

HELP = "space: play/pause   ←/→: step   ↑/↓: speed   Home/End: start/end"


class Player:
    """Keyboard and timer control of a function `show(t)` for t in 0 … n-1."""

    def __init__(self, fig, n: int, show: Callable[[int], None], interval_ms: int = 120):
        self.fig, self.n, self.show = fig, n, show
        self.t, self.playing, self.interval = 0, False, interval_ms
        self.timer = fig.canvas.new_timer(interval=interval_ms)
        self.timer.add_callback(self._tick)
        fig.canvas.mpl_connect("key_press_event", self._key)
        self.goto(0)

    def goto(self, t: int):
        self.t = max(0, min(self.n - 1, t))
        self.show(self.t)
        self.fig.canvas.draw_idle()

    def _tick(self):
        if self.t >= self.n - 1:
            self.playing = False
            self.timer.stop()
            return
        self.goto(self.t + 1)

    def _key(self, event):
        k = event.key
        if k == " ":
            self.playing = not self.playing
            (self.timer.start if self.playing else self.timer.stop)()
        elif k == "right":
            self.goto(self.t + 1)
        elif k == "left":
            self.goto(self.t - 1)
        elif k == "home":
            self.goto(0)
        elif k == "end":
            self.goto(self.n - 1)
        elif k in ("up", "down"):
            self.interval = int(max(15, min(2000, self.interval * (0.6 if k == "up" else 1.6))))
            self.timer.interval = self.interval


class Incremental:
    """Calls `solve(t, previous)` lazily and caches the results; a NotImplementedError (a task not
    done yet) is remembered as None, so the player simply leaves that layer out."""

    def __init__(self, solve: Callable[[int, Optional[object]], object]):
        self.solve, self.cache, self.missing = solve, {}, False

    def __call__(self, t: int):
        if self.missing:
            return None
        if t not in self.cache:
            prev = self(t - 1) if t > 0 else None
            try:
                self.cache[t] = self.solve(t, prev)
            except NotImplementedError:
                self.missing = True
                return None
        return self.cache[t]


def _heading_marker(ax, color, size=0.35):
    (line,) = ax.plot([], [], "-", color=color, linewidth=2.5, solid_capstyle="round")
    (dot,) = ax.plot([], [], "o", color=color, markersize=7, markerfacecolor="white", markeredgewidth=2)
    def update(x, y, theta):
        dot.set_data([x], [y])
        line.set_data([x, x + size * np.cos(theta)], [y, y + size * np.sin(theta)])
    return update


# --------------------------------------------------------------------------------------------------
def play_warehouse(world, true_poses, odometry, observations, backend, step_size: float):
    """The floor, step by step. backend: a WarehouseBackend (its residuals may be unfinished)."""
    from .backend_2d import dead_reckoning

    truth = np.array([(p.x, p.y, p.theta) for p in true_poses])
    n = len(truth)
    start = true_poses[0]
    tags = world.landmarks
    obs_by_t = [[] for _ in range(n)]
    for o in observations:
        obs_by_t[o.pose_index].append(o)

    # dead reckoning: all at once (it needs `compose`); SLAM: incrementally (it needs the residuals too)
    try:
        dr = dead_reckoning(start, odometry)
    except NotImplementedError:
        dr = None

    def solve(t, prev):
        odo_t = odometry[:t]
        obs_t = [o for o in observations if o.pose_index <= t]
        ids = sorted({o.landmark_id for o in obs_t})
        initial = None
        if prev is not None:  # warm start: the previous estimate, extended by one dead-reckoning step
            prev_poses, prev_lm = prev
            o = odo_t[-1]
            new_pose = compose(*prev_poses[-1], o.dx, o.dy, o.dtheta)
            guess_poses = np.vstack([prev_poses, new_pose])
            guess_lm = np.zeros((len(ids), 2))
            for k, lm in enumerate(ids):
                if lm in prev_lm:
                    guess_lm[k] = prev_lm[lm]
                else:  # first seen now: place it where this sighting says, from the new pose
                    first = next(ob for ob in obs_by_t[t] if ob.landmark_id == lm)
                    guess_lm[k] = landmark_from_observation(*new_pose, first.range, first.bearing)
            initial = (guess_poses, guess_lm)
        poses, lm, _ = backend.optimize(start, odo_t, obs_t, ids, initial=initial)
        return poses, lm

    slam = Incremental(solve)

    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 6.8), gridspec_kw={"width_ratios": [2.2, 1]})
    draw_warehouse(ax, world)
    (truth_line,) = ax.plot([], [], "--", color=GREEN, linewidth=1.4, label="true path")
    rays = LineCollection([], colors=BLUE, linewidths=0.8, alpha=0.6, label="sightings")
    ax.add_collection(rays)
    (dr_line,) = ax.plot([], [], "-", color=RED, linewidth=1.6, label="dead reckoning")
    (slam_line,) = ax.plot([], [], "-", color=BLUE, linewidth=1.6, label="SLAM")
    slam_tags = ax.scatter([], [], marker="x", s=45, color=BLUE, zorder=6, label="tags (SLAM)")
    robot_true = _heading_marker(ax, GREEN)
    robot_dr = _heading_marker(ax, RED)
    robot_slam = _heading_marker(ax, BLUE)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=6, fontsize=9, frameon=False)
    title = ax.set_title("")

    steps = np.arange(n)
    (err_dr,) = ax2.plot([], [], color=RED, label="dead reckoning")
    (err_slam,) = ax2.plot([], [], color=BLUE, label="SLAM (current estimate of every pose so far)")
    ax2.set_xlim(0, n - 1)
    ax2.set_ylim(0, 1.0)
    ax2.set_xlabel("step")
    ax2.set_ylabel("position error (m)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=8, loc="upper left")
    ax2.set_title("how far from the truth")
    note = ax2.text(0.5, 0.5, "", transform=ax2.transAxes, ha="center", va="center", fontsize=10, color=GREY)
    fig.text(0.5, 0.005, HELP, ha="center", fontsize=9, color=GREY)
    fig.tight_layout(rect=(0, 0.03, 1, 1))

    dr_err = np.hypot(*(dr[:, :2] - truth[:, :2]).T) if dr is not None else None
    parts = []

    def show(t):
        parts.clear()
        truth_line.set_data(truth[: t + 1, 0], truth[: t + 1, 1])
        robot_true(*truth[t])
        x, y = truth[t, 0], truth[t, 1]
        rays.set_segments([[(x, y), tags[o.landmark_id]] for o in obs_by_t[t]])
        ymax = 0.3
        if dr is not None:
            dr_line.set_data(dr[: t + 1, 0], dr[: t + 1, 1])
            robot_dr(*dr[t])
            err_dr.set_data(steps[: t + 1], dr_err[: t + 1])
            ymax = max(ymax, dr_err[: t + 1].max() * 1.1)
        else:
            parts.append("dead reckoning: implement compose()")
        est = slam(t)
        if est is not None:
            poses, lm = est
            slam_line.set_data(poses[:, 0], poses[:, 1])
            robot_slam(*poses[-1])
            slam_tags.set_offsets(np.array(list(lm.values())).reshape(-1, 2))
            e = np.hypot(*(poses[:, :2] - truth[: t + 1, :2]).T)
            err_slam.set_data(steps[: t + 1], e)
            ymax = max(ymax, e.max() * 1.1)
        else:
            parts.append("SLAM: implement the residuals")
        ax2.set_ylim(0, ymax)
        note.set_text("not yet:\n" + "\n".join(parts) if parts else "")
        title.set_text(f"step {t} of {n - 1}: {t * step_size:.1f} m driven, {len(obs_by_t[t])} tags in view")

    player = Player(fig, n, show)
    plt.show()
    return player


# --------------------------------------------------------------------------------------------------
def play_aisle(posts, true_x, odometry, observations, backend, half_width: float):
    """The aisle, step by step."""
    true_x = np.asarray(true_x, dtype=float)
    n = len(true_x)
    start = true_x[0]
    obs_by_t = [[] for _ in range(n)]
    for o in observations:
        obs_by_t[o.pose_index].append(o)
    dr = backend.dead_reckoning(start, odometry)
    dr_err = np.abs(dr - true_x)

    def solve(t, prev):
        obs_t = [o for o in observations if o.pose_index <= t]
        x, p, _ = backend.optimize(start, odometry[:t], obs_t)
        return x, p

    slam = Incremental(solve)

    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 7.2), gridspec_kw={"height_ratios": [1.1, 1]})
    L = max(p[0] for p in posts.values()) + 1
    from matplotlib.patches import Rectangle

    ax.add_patch(Rectangle((-1, half_width), L + 2, 0.6, facecolor="#d9dce2", edgecolor="none"))
    ax.add_patch(Rectangle((-1, -half_width - 0.6), L + 2, 0.6, facecolor="#d9dce2", edgecolor="none"))
    ax.scatter([p[0] for p in posts.values()], [p[1] for p in posts.values()], marker="*", s=90, color=GREEN, zorder=5, label="posts (true)")
    est_posts = ax.scatter([], [], marker="x", s=60, color=BLUE, zorder=6, label="posts (SLAM)")
    rays = LineCollection([], colors=BLUE, linewidths=0.8, alpha=0.6, label="sightings")
    ax.add_collection(rays)
    (true_pts,) = ax.plot([], [], "o", color=GREEN, markersize=4, label="true positions")
    (dr_pts,) = ax.plot([], [], "x", color=RED, markersize=5, label="dead reckoning")
    (slam_pts,) = ax.plot([], [], "s", color=BLUE, markersize=4, label="SLAM")
    (robot,) = ax.plot([], [], "o", color=GREEN, markersize=9, markerfacecolor="white", markeredgewidth=2)
    ax.set_xlim(-1, L + 1)
    ax.set_ylim(-half_width - 0.8, half_width + 0.8)
    ax.set_aspect("equal")
    ax.set_yticks([])
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=6, fontsize=9, frameon=False)
    title = ax.set_title("")
    steps = np.arange(n)
    (err_dr,) = ax2.plot([], [], color=RED, label="dead reckoning")
    (err_slam,) = ax2.plot([], [], color=BLUE, label="SLAM (current estimate of every position so far)")
    ax2.set_xlim(0, n - 1)
    ax2.set_xlabel("step")
    ax2.set_ylabel("position error (m)")
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=8, loc="upper left")
    note = ax2.text(0.5, 0.5, "", transform=ax2.transAxes, ha="center", va="center", fontsize=10, color=GREY)
    fig.text(0.5, 0.005, HELP, ha="center", fontsize=9, color=GREY)
    fig.tight_layout(rect=(0, 0.03, 1, 1))

    def show(t):
        x = true_x[t]
        true_pts.set_data(true_x[: t + 1], np.zeros(t + 1))
        robot.set_data([x], [0])
        rays.set_segments([[(x, 0), posts[o.landmark_id]] for o in obs_by_t[t]])
        dr_pts.set_data(dr[: t + 1], np.full(t + 1, -0.25))
        err_dr.set_data(steps[: t + 1], dr_err[: t + 1])
        ymax = max(0.1, dr_err[: t + 1].max() * 1.1)
        est = slam(t)
        if est is not None:
            ex, ep = est
            slam_pts.set_data(ex, np.full(t + 1, 0.25))
            est_posts.set_offsets(np.array([(ep[k], posts[k][1]) for k in ep]).reshape(-1, 2))
            e = np.abs(ex - true_x[: t + 1])
            err_slam.set_data(steps[: t + 1], e)
            ymax = max(ymax, e.max() * 1.1)
        note.set_text("" if est is not None else "not yet:\nSLAM: implement the residuals")
        ax2.set_ylim(0, ymax)
        title.set_text(f"step {t} of {n - 1}: {len(obs_by_t[t])} posts in view")

    player = Player(fig, n, show)
    plt.show()
    return player
