#!/usr/bin/env python3
"""Plot the latest Stage 3 recorded trajectory and altitude profile."""

import argparse
import csv
import math
import shutil
from datetime import datetime
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_ARTIFACTS_ROOT = REPO_ROOT / "runtime_artifacts"


def require_runtime_artifact_path(path):
    resolved = path.resolve()
    try:
        resolved.relative_to(RUNTIME_ARTIFACTS_ROOT)
    except ValueError:
        raise ValueError(
            "Output directory must be under {}: {}".format(
                RUNTIME_ARTIFACTS_ROOT, resolved
            )
        )
    return resolved


PHASE_STYLE = {
    "preflight": ("Preflight / ground", "#7f8c8d"),
    "ascent": ("Segmented ascent", "#34495e"),
    "entry": ("ENTRY_GATE transit", "#16a085"),
    "layer_26": ("26 m inspection", "#1565c0"),
    "transition": ("26 ↔ 22 m transition", "#f39c12"),
    "layer_22": ("22 m inspection", "#2e7d32"),
    "exit_gate": ("GO_TO_EXIT_GATE", "#8e44ad"),
    "return_home": ("RETURN_HOME / landing", "#c62828"),
}


def latest_csv():
    candidates = list(
        RUNTIME_ARTIFACTS_ROOT
        .glob("sector_inspection_control_*/control.csv")
    )
    if not candidates:
        raise FileNotFoundError(
            "No control CSV found under runtime_artifacts/sector_inspection_control_*"
        )
    return max(candidates, key=lambda path: path.stat().st_mtime)


def finite(value):
    parsed = float(value)
    if not math.isfinite(parsed):
        raise ValueError("non-finite value")
    return parsed


def read_csv(path):
    samples = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {"sim_time", "state", "layer", "x", "y", "z"}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "CSV is missing required fields: {}".format(
                    ", ".join(sorted(missing))
                )
            )
        for row in reader:
            try:
                samples.append(
                    {
                        "time": finite(row["sim_time"]),
                        "state": row["state"],
                        "layer": int(row["layer"]),
                        "x": finite(row["x"]),
                        "y": finite(row["y"]),
                        "z": finite(row["z"]),
                    }
                )
            except (TypeError, ValueError):
                continue
    if len(samples) < 2:
        raise ValueError("CSV does not contain enough finite trajectory data")
    return samples


def phase(sample):
    state = sample["state"]
    if state == "WAIT_INPUTS":
        return "preflight"
    if state in {"STAGING_POINT", "SEGMENTED_CLIMB"}:
        return "ascent"
    if state == "ENTRY_GATE_TRANSIT":
        return "entry"
    if state == "LAYER_TRANSITION":
        return "transition"
    if state == "GO_TO_EXIT_GATE":
        return "exit_gate"
    if state in {"RETURN_HOME", "DONE"}:
        return "return_home"
    return "layer_26" if sample["layer"] == 0 else "layer_22"


def phase_runs(samples):
    labels = [phase(sample) for sample in samples]
    start = 0
    for index in range(1, len(samples)):
        if labels[index] == labels[start]:
            continue
        yield labels[start], max(0, start - 1), index - 1
        start = index
    yield labels[start], max(0, start - 1), len(samples) - 1


def first_state_times(samples):
    wanted = {
        "LAYER_TRANSITION",
        "GO_TO_EXIT_GATE",
        "RETURN_HOME",
        "DONE",
    }
    events = {}
    for sample in samples:
        state = sample["state"]
        if state in wanted and state not in events:
            events[state] = sample["time"]
    return events


def equal_metric_limits(ax, xs, ys, zs):
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    z_min, z_max = min(zs), max(zs)
    span = max(x_max - x_min, y_max - y_min, z_max - z_min, 1.0)
    margin = 0.06 * span
    half = 0.5 * span + margin
    x_mid = 0.5 * (x_min + x_max)
    y_mid = 0.5 * (y_min + y_max)
    z_mid = 0.5 * (z_min + z_max)
    ax.set_xlim(x_mid - half, x_mid + half)
    ax.set_ylim(y_mid - half, y_mid + half)
    ax.set_zlim(z_mid - half, z_mid + half)
    # Matplotlib 3.1 uses a 4:4:3 display box. Compensate the projection so
    # one metre has the same visual scale on X, Y and Z.
    original_projection = ax.get_proj
    ax.get_proj = lambda: np.dot(
        original_projection(), np.diag([1.0, 1.0, 4.0 / 3.0, 1.0])
    )


def plot(samples, source, output, tower_x, tower_y, radius):
    times = np.array([sample["time"] for sample in samples])
    times -= times[0]
    xs = np.array([sample["x"] for sample in samples])
    ys = np.array([sample["y"] for sample in samples])
    zs = np.array([sample["z"] for sample in samples])

    figure = plt.figure(figsize=(16, 8.5), constrained_layout=True)
    trajectory = figure.add_subplot(1, 2, 1, projection="3d")
    legend_seen = set()
    for label, start, end in phase_runs(samples):
        display, color = PHASE_STYLE[label]
        trajectory.plot(
            xs[start : end + 1],
            ys[start : end + 1],
            zs[start : end + 1],
            color=color,
            linewidth=1.7,
            label=display if label not in legend_seen else None,
        )
        legend_seen.add(label)

    theta = np.linspace(0.0, 2.0 * math.pi, 241)
    for height, color in ((26.0, "#1565c0"), (22.0, "#2e7d32")):
        trajectory.plot(
            tower_x + radius * np.cos(theta),
            tower_y + radius * np.sin(theta),
            np.full_like(theta, height),
            color=color,
            linestyle=":",
            linewidth=0.9,
            alpha=0.55,
        )
    trajectory.plot(
        [tower_x, tower_x],
        [tower_y, tower_y],
        [0.0, max(28.0, float(zs.max()))],
        color="#555555",
        linestyle="--",
        linewidth=1.0,
        label="Tower centre axis",
    )
    trajectory.scatter(
        xs[0], ys[0], zs[0], color="#00a000", s=55, marker="o",
        edgecolor="white", linewidth=0.7, label="Start"
    )
    trajectory.scatter(
        xs[-1], ys[-1], zs[-1], color="#e00000", s=65, marker="x",
        linewidth=2.0, label="End"
    )
    equal_metric_limits(
        trajectory,
        np.append(xs, [tower_x - radius, tower_x + radius]),
        np.append(ys, [tower_y - radius, tower_y + radius]),
        np.append(zs, [0.0, 28.0]),
    )
    trajectory.view_init(elev=27.0, azim=-57.0)
    trajectory.set_xlabel("X / East (m)", labelpad=8)
    trajectory.set_ylabel("Y / North (m)", labelpad=8)
    trajectory.set_zlabel("Z / Up (m)", labelpad=7)
    trajectory.set_title(
        "Actual recorded 3D trajectory\n"
        "FAST-LIO mission odometry, camera_init / ENU"
    )
    trajectory.legend(loc="upper left", fontsize=8)

    altitude = figure.add_subplot(1, 2, 2)
    altitude.plot(times, zs, color="#222222", linewidth=1.0)
    event_colors = {
        "LAYER_TRANSITION": "#f39c12",
        "GO_TO_EXIT_GATE": "#8e44ad",
        "RETURN_HOME": "#c62828",
        "DONE": "#27ae60",
    }
    events = first_state_times(samples)
    for state, stamp in events.items():
        altitude.axvline(
            stamp - samples[0]["time"],
            color=event_colors[state],
            linestyle="--",
            linewidth=1.2,
            label=state,
        )
    if "DONE" not in events and zs[-1] < 0.5:
        altitude.axvline(
            times[-1],
            color=event_colors["DONE"],
            linestyle="--",
            linewidth=1.2,
            label="END / LANDED",
        )
    altitude.set_xlabel("Mission elapsed time (s)")
    altitude.set_ylabel("Z / Up (m)")
    altitude.set_title("Actual altitude profile")
    altitude.grid(alpha=0.25)
    altitude.legend(loc="lower center", fontsize=9)

    figure.suptitle(
        "Stage 3 flight evidence — {}".format(source.name), fontsize=13
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=(
            RUNTIME_ARTIFACTS_ROOT
            / "inspection_trajectory_{}".format(datetime.now().strftime("%Y%m%d_%H%M%S"))
        ),
    )
    parser.add_argument("--tower-x", type=float, default=-10.0551)
    parser.add_argument("--tower-y", type=float, default=19.7104)
    parser.add_argument("--radius", type=float, default=12.5)
    args = parser.parse_args()

    source = args.csv.resolve() if args.csv else latest_csv().resolve()
    samples = read_csv(source)
    output = require_runtime_artifact_path(args.output_dir) / (
        "inspection_actual_trajectory_{}.png".format(source.stem)
    )
    plot(
        samples,
        source,
        output,
        args.tower_x,
        args.tower_y,
        args.radius,
    )
    latest = output.parent / "latest_inspection_trajectory.png"
    shutil.copy2(str(output), str(latest))
    print(output)
    print(latest)


if __name__ == "__main__":
    main()
