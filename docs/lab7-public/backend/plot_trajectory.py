"""Save 3D and top-down flight plots, or regenerate them from an NPZ/JSON run."""

import argparse
import json
from pathlib import Path

import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure

from backend.course import Course, Gate


def save_trajectory_plot(course, positions, path, *, top_down=False):
    """Plot one drone and gate openings, in 3D or projected onto the XY plane."""
    positions = np.asarray(positions)
    valid = positions[np.isfinite(positions).all(axis=-1)]
    dimensions = 2 if top_down else 3
    fig = Figure(figsize=(9, 8) if top_down else (11, 8), layout="constrained")
    FigureCanvasAgg(fig)  # Save headlessly, including on machines without a display.
    ax = fig.add_subplot() if top_down else fig.add_subplot(projection="3d")
    ax.plot(*positions[:, :dimensions].T, color="#244b75", linewidth=1.8, label="Trajectory")
    marker_options = {} if top_down else {"depthshade": False}
    ax.scatter(*valid[0, :dimensions], color="tab:green", s=55, label="Start", **marker_options)
    ax.scatter(
        *valid[-1, :dimensions], color="tab:red", marker="X", s=65, label="End", **marker_options
    )

    w, h = course.opening_width / 2, course.opening_height / 2
    rectangle = np.array([[0, -w, -h], [0, w, -h], [0, w, h], [0, -w, h], [0, -w, -h]])
    bounds = [valid]
    for index, gate in enumerate(course.gates, 1):
        yaw = np.deg2rad(gate.yaw_degrees)
        c, s = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        corners = rectangle @ rotation.T + gate.center
        ax.plot(*corners[:, :dimensions].T, color=gate.color, linewidth=3)
        label = f"{index}: {gate.name}"
        if top_down:
            ax.annotate(
                label, gate.center[:2], xytext=(8, 8), textcoords="offset points", fontsize=9
            )
        else:
            ax.text(*corners[2], f"  {label}", fontsize=9)
        bounds.append(corners)

    points = np.concatenate(bounds)
    low, high = points.min(axis=0), points.max(axis=0)
    low[2] = min(low[2], 0)  # Include ground level for altitude context in 3D.
    span = np.maximum(high - low, 0.5)
    low, high = low - 0.08 * span, high + 0.08 * span
    ax.set(xlim=(low[0], high[0]), ylim=(low[1], high[1]), xlabel="X [m]", ylabel="Y [m]")
    ax.set_title(f"Drone trajectory — {course.name}" + (" (top view)" if top_down else ""))
    if top_down:
        ax.set_aspect("equal")
        ax.grid(alpha=0.25)
        ax.set_axisbelow(True)
    else:
        ax.set(zlim=(low[2], high[2]), zlabel="Z [m]")
        ax.set_box_aspect(high - low)  # One metre has the same scale on every axis.
        ax.view_init(elev=27, azim=-37)
    ax.legend(loc="upper left")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=180)
    return path


def save_trajectory_plots(course, positions, path):
    """Save the 3D PNG and a second, top-down PNG with a '-top' suffix."""
    path = Path(path)
    top_path = path.with_stem(path.stem + "-top")
    save_trajectory_plot(course, positions, path)
    save_trajectory_plot(course, positions, top_path, top_down=True)
    return path, top_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path, help="Run NPZ, with its JSON metrics alongside")
    parser.add_argument("--output", type=Path, help="PNG path; defaults to the NPZ filename stem")
    args = parser.parse_args()
    data = json.loads(args.trajectory.with_suffix(".json").read_text())["course"]
    course = Course(**{**data, "gates": tuple(Gate(**gate) for gate in data["gates"])})
    with np.load(args.trajectory) as run:
        paths = save_trajectory_plots(
            course, run["position"][:, 0], args.output or args.trajectory.with_suffix(".png")
        )
    for path in paths:
        print(f"Trajectory plot: {path}")
