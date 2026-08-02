#!/usr/bin/env python3
"""Plot the permanent Stage-5 recorder CSV in the shared world/ENU frame."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401; registers projection


COLORS = {1: "#1565c0", 2: "#e76f00", 3: "#2e8b57"}


def finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def load(path):
    samples = {1: [], 2: [], 3: []}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamp = finite(row.get("sim_time"))
            if stamp is None:
                continue
            for uid in samples:
                xyz = tuple(finite(row.get("u{}_{}".format(uid, axis)))
                            for axis in "xyz")
                if any(value is None for value in xyz):
                    continue
                samples[uid].append((stamp,) + xyz)
    if any(len(records) < 2 for records in samples.values()):
        raise RuntimeError("each UAV needs at least two finite trajectory samples")
    return samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--output-3d", type=Path)
    args = parser.parse_args()
    samples = load(args.csv)
    common_start = min(records[0][0] for records in samples.values())

    figure, (xy_axis, altitude_axis) = plt.subplots(
        1, 2, figsize=(14, 6.5), constrained_layout=True)
    for uid, records in samples.items():
        elapsed = [row[0] - common_start for row in records]
        x = [row[1] for row in records]
        y = [row[2] for row in records]
        z = [row[3] for row in records]
        color = COLORS[uid]
        xy_axis.plot(x, y, color=color, linewidth=1.35, label="UAV{}".format(uid))
        xy_axis.scatter(x[0], y[0], color=color, marker="o", s=40)
        xy_axis.scatter(x[-1], y[-1], color=color, marker="X", s=55)
        altitude_axis.plot(elapsed, z, color=color, linewidth=1.2,
                           label="UAV{}".format(uid))

    tower_x, tower_y = -10.0551, 19.7104
    xy_axis.scatter(tower_x, tower_y, color="#333333", marker="+", s=110,
                    linewidth=2.0, label="tower centre")
    xy_axis.set_aspect("equal", adjustable="box")
    xy_axis.set_xlabel("world X / East (m)")
    xy_axis.set_ylabel("world Y / North (m)")
    xy_axis.set_title("Stage-5 three-UAV XY trajectories")
    xy_axis.grid(True, alpha=0.25)
    xy_axis.legend()
    altitude_axis.set_xlabel("elapsed simulation time (s)")
    altitude_axis.set_ylabel("world Z / Up (m)")
    altitude_axis.set_title("Altitude history")
    altitude_axis.grid(True, alpha=0.25)
    altitude_axis.legend()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(str(args.output), dpi=160)
    plt.close(figure)

    if args.output_3d is not None:
        figure_3d = plt.figure(figsize=(10, 8), constrained_layout=True)
        axis_3d = figure_3d.add_subplot(111, projection="3d")
        for uid, records in samples.items():
            x = [row[1] for row in records]
            y = [row[2] for row in records]
            z = [row[3] for row in records]
            color = COLORS[uid]
            axis_3d.plot(x, y, z, color=color, linewidth=1.35,
                         label="UAV{}".format(uid))
            axis_3d.scatter(x[0], y[0], z[0], color=color, marker="o", s=35)
            axis_3d.scatter(x[-1], y[-1], z[-1], color=color,
                            marker="X", s=50)
        maximum_height = max(
            row[3] for records in samples.values() for row in records)
        axis_3d.plot([tower_x, tower_x], [tower_y, tower_y],
                     [0.0, max(3.0, maximum_height)], color="#333333",
                     linewidth=2.0, label="tower centreline")
        axis_3d.set_xlabel("world X / East (m)")
        axis_3d.set_ylabel("world Y / North (m)")
        axis_3d.set_zlabel("world Z / Up (m)")
        axis_3d.set_title("Stage-5 three-UAV 3D trajectories")
        axis_3d.legend()
        args.output_3d.parent.mkdir(parents=True, exist_ok=True)
        figure_3d.savefig(str(args.output_3d), dpi=160)
        plt.close(figure_3d)


if __name__ == "__main__":
    main()
