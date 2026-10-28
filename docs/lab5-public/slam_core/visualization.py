"""Plots: the warehouse with the three trajectories (truth, dead reckoning, SLAM), and the aisle."""

import math
from typing import Dict, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Rectangle

from .models import Pose2D
from .world import Warehouse

GREEN, RED, BLUE, GREY = "#17914a", "#d23c3c", "#1c6ed0", "#9aa1ae"


def draw_warehouse(ax, world: Warehouse, landmarks: bool = True):
    for c, r in world.walls:
        ax.add_patch(Rectangle((c, r), 1, 1, facecolor="#d9dce2", edgecolor="#c0c4cc", linewidth=0.5))
    if landmarks:
        xs = [p[0] for p in world.landmarks.values()]
        ys = [p[1] for p in world.landmarks.values()]
        ax.scatter(xs, ys, marker="*", s=70, color=GREEN, zorder=5, label="tags (true)")
    sx, sy = world.cell_centre(world.start_cell)
    ax.annotate("start", (sx, sy), xytext=(4, 6), textcoords="offset points", fontsize=8, color="black")
    ax.set_xlim(-0.2, world.width + 0.2)
    ax.set_ylim(-0.2, world.height + 0.2)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])


def draw_path(ax, poses: np.ndarray, color: str, label: str, style: str = "-", headings: bool = False, **kw):
    ax.plot(poses[:, 0], poses[:, 1], style, color=color, label=label, linewidth=1.6, **kw)
    ax.plot(poses[-1, 0], poses[-1, 1], "o", color=color, markersize=5)
    if headings:
        step = max(1, len(poses) // 40)
        p = poses[::step]
        ax.quiver(p[:, 0], p[:, 1], np.cos(p[:, 2]), np.sin(p[:, 2]), color=color, angles="xy", scale_units="xy", scale=3, width=0.003)


def plot_warehouse_results(
    world: Warehouse,
    true_poses: Sequence[Pose2D],
    dr_poses: np.ndarray,
    est_poses: Optional[np.ndarray] = None,
    est_landmarks: Optional[Dict[int, Tuple[float, float]]] = None,
    title: str = "",
    save_path: Optional[str] = None,
):
    truth = np.array([(p.x, p.y, p.theta) for p in true_poses])
    fig, (ax, ax2) = plt.subplots(1, 2, figsize=(14, 6.5), gridspec_kw={"width_ratios": [2.2, 1]})
    draw_warehouse(ax, world)
    draw_path(ax, truth, GREEN, "true path", "--")
    draw_path(ax, dr_poses, RED, "dead reckoning (odometry only)", "-", headings=True, alpha=0.9)
    if est_poses is not None:
        draw_path(ax, est_poses, BLUE, "SLAM", "-", headings=True)
    if est_landmarks:
        xs = [p[0] for p in est_landmarks.values()]
        ys = [p[1] for p in est_landmarks.values()]
        ax.scatter(xs, ys, marker="x", s=50, color=BLUE, zorder=6, label="tags (SLAM)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.02), ncol=3, fontsize=9, frameon=False)
    ax.set_title(title or "the warehouse")

    # the position error along the route
    steps = np.arange(len(truth))
    ax2.plot(steps, np.hypot(*(dr_poses[:, :2] - truth[:, :2]).T), color=RED, label="dead reckoning")
    if est_poses is not None:
        ax2.plot(steps, np.hypot(*(est_poses[:, :2] - truth[:, :2]).T), color=BLUE, label="SLAM")
    ax2.set_xlabel("step")
    ax2.set_ylabel("position error (m)")
    ax2.set_ylim(bottom=0)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=9)
    ax2.set_title("how far from the truth")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=130)
        print(f"saved {save_path}")
    return fig


def plot_aisle_results(
    posts: Dict[int, Tuple[float, float]],
    true_x: np.ndarray,
    dr_x: np.ndarray,
    est_x: Optional[np.ndarray] = None,
    est_posts: Optional[Dict[int, float]] = None,
    aisle_half_width: float = 1.0,
    sigma_odom: Optional[float] = None,
    other_dr_errors: Sequence[np.ndarray] = (),
    save_path: Optional[str] = None,
):
    fig, (ax, ax2) = plt.subplots(2, 1, figsize=(12, 7), gridspec_kw={"height_ratios": [1.1, 1]})
    # the aisle: the shelves as two grey bands, the posts as stars
    L = max(max(p[0] for p in posts.values()), true_x.max()) + 1
    for sign in (1, -1):
        ax.add_patch(Rectangle((-1, sign * aisle_half_width if sign > 0 else -aisle_half_width - 0.6), L + 2, 0.6, facecolor="#d9dce2", edgecolor="none"))
    ax.scatter([p[0] for p in posts.values()], [p[1] for p in posts.values()], marker="*", s=90, color=GREEN, zorder=5, label="posts (true)")
    if est_posts:
        ax.scatter([est_posts[k] for k in est_posts], [posts[k][1] for k in est_posts], marker="x", s=60, color=BLUE, zorder=6, label="posts (SLAM)")
    ax.plot(true_x, np.zeros_like(true_x), "o", color=GREEN, markersize=4, label="true positions")
    ax.plot(dr_x, np.full_like(dr_x, -0.25), "x", color=RED, markersize=5, label="dead reckoning")
    if est_x is not None:
        ax.plot(est_x, np.full_like(est_x, 0.25), "s", color=BLUE, markersize=4, label="SLAM")
    ax.set_xlim(-1, L + 1)
    ax.set_ylim(-aisle_half_width - 0.8, aisle_half_width + 0.8)
    ax.set_aspect("equal")
    ax.set_yticks([])
    ax.set_xlabel("x along the aisle (m)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.28), ncol=6, fontsize=9, frameon=False)
    ax.set_title("the aisle: to the end and back")
    steps = np.arange(len(true_x))
    for k, e in enumerate(other_dr_errors):  # the same drive with other noise draws
        ax2.plot(steps, e, color=RED, alpha=0.15, linewidth=1, label="dead reckoning, other runs" if k == 0 else None)
    if sigma_odom is not None:  # the typical size of a random walk after t steps
        ax2.plot(steps, sigma_odom * np.sqrt(steps), ":", color="black", label="σ·√steps")
    ax2.plot(steps, np.abs(dr_x - true_x), color=RED, label="dead reckoning")
    if est_x is not None:
        ax2.plot(steps, np.abs(est_x - true_x), color=BLUE, label="SLAM")
    ax2.set_xlabel("step")
    ax2.set_ylabel("position error (m)")
    ax2.set_ylim(bottom=0)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(fontsize=9)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=130)
        print(f"saved {save_path}")
    return fig


def plot_jacobian_pattern(jac, n_poses: int, n_odom: int, save_path: Optional[str] = None):
    """The sparsity pattern of the Jacobian at the solution: one row per residual, one column per unknown."""
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.spy(jac, markersize=0.6, color=BLUE, aspect="auto")
    ax.axvline(3 * n_poses - 0.5, color=GREY, linewidth=0.8)
    ax.axhline(3 + 3 * n_odom - 0.5, color=GREY, linewidth=0.8)
    ax.text(1.5 * n_poses, -8, "pose columns", ha="center", fontsize=9)
    ax.text(3 * n_poses + (jac.shape[1] - 3 * n_poses) / 2, -8, "landmark columns", ha="center", fontsize=9)
    ax.set_ylabel("residuals: prior, odometry (top), observations (bottom)")
    ax.set_title(f"non-zeros: {jac.nnz} of {jac.shape[0]} × {jac.shape[1]}")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=130)
        print(f"saved {save_path}")
    return fig
