#!/usr/bin/env python3
"""Plot and summarize one low-altitude flight against its exact world file."""

import argparse
import csv
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


NOMINAL_HEIGHT = 3.0
ORBIT_RADIUS = 12.5
EGO_INFLATION = 0.4
EGO_TRAJECTORY_CLEARANCE = 0.5


def finite(row, name):
    value = float(row[name])
    if not math.isfinite(value):
        raise ValueError(name)
    return value


def read_samples(path):
    samples = []
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        required = {
            "sim_time",
            "state",
            "target_id",
            "target_x",
            "target_y",
            "target_z",
            "x",
            "y",
            "z",
            "actual_yaw",
            "horizontal_speed",
            "yaw_mode",
            "ref_x",
            "ref_y",
            "ref_z",
            "altitude_policy",
            "horizontal_path_available",
            "vertical_escape_allowed",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(
                "CSV missing fields: {}".format(", ".join(sorted(missing)))
            )
        for row in reader:
            try:
                samples.append(
                    {
                        "time": finite(row, "sim_time"),
                        "state": row["state"],
                        "target_id": row["target_id"],
                        "target_x": finite(row, "target_x"),
                        "target_y": finite(row, "target_y"),
                        "target_z": finite(row, "target_z"),
                        "x": finite(row, "x"),
                        "y": finite(row, "y"),
                        "z": finite(row, "z"),
                        "yaw": finite(row, "actual_yaw"),
                        "speed": finite(row, "horizontal_speed"),
                        "yaw_mode": row["yaw_mode"],
                        "ref_x": finite(row, "ref_x"),
                        "ref_y": finite(row, "ref_y"),
                        "ref_z": finite(row, "ref_z"),
                        "policy": row["altitude_policy"],
                        "horizontal": int(row["horizontal_path_available"]),
                        "vertical": int(row["vertical_escape_allowed"]),
                        "replans": int(row.get("low_ingress_replans", "0")),
                        "waypoint": int(row.get("waypoint", "0")),
                    }
                )
            except (TypeError, ValueError):
                continue
    if len(samples) < 2:
        raise ValueError("CSV has fewer than two finite samples")
    return samples


def pose_values(element):
    if element is None or not element.text:
        return None
    values = [float(value) for value in element.text.split()]
    return values if len(values) >= 3 else None


def read_world(path):
    root = ET.parse(str(path)).getroot()
    world = root.find(".//world")
    if world is None:
        raise ValueError("SDF does not contain a world")
    state = world.find("state")
    state_poses = {}
    if state is not None:
        for model in state.findall("model"):
            values = pose_values(model.find("pose"))
            if values:
                state_poses[model.get("name", "")] = values

    models = {}
    for model in world.findall("model"):
        name = model.get("name", "")
        values = state_poses.get(name) or pose_values(model.find("pose"))
        if not values:
            continue
        item = {"name": name, "x": values[0], "y": values[1], "z": values[2]}
        radius = model.findtext(
            "./link/collision/geometry/cylinder/radius"
        )
        if radius is not None:
            item["radius"] = float(radius)
        models[name] = item
    for name, values in state_poses.items():
        if name not in models:
            models[name] = {
                "name": name,
                "x": values[0],
                "y": values[1],
                "z": values[2],
            }
    tower = models.get("radio_tower")
    if tower is None:
        raise ValueError("radio_tower is absent from worksite.world")
    cylinders = [
        item
        for name, item in sorted(models.items())
        if name.startswith("moving_cylinder") and "radius" in item
    ]
    if len(cylinders) < 2:
        raise ValueError("fewer than two cylinder obstacles in world")
    return tower, cylinders


def path_length(samples, x_name="x", y_name="y", z_name="z"):
    return sum(
        math.sqrt(
            (right[x_name] - left[x_name]) ** 2
            + (right[y_name] - left[y_name]) ** 2
            + (right[z_name] - left[z_name]) ** 2
        )
        for left, right in zip(samples, samples[1:])
    )


def angle_error(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, int(round(fraction * (len(ordered) - 1))))
    return ordered[index]


def tangent_yaw_errors(samples):
    errors = []
    for left, right in zip(samples, samples[1:]):
        dx = right["x"] - left["x"]
        dy = right["y"] - left["y"]
        if math.hypot(dx, dy) < 0.02 or right["speed"] < 0.08:
            continue
        errors.append(abs(angle_error(right["yaw"], math.atan2(dy, dx))))
    return errors


def tower_yaw_errors(samples, tower):
    errors = []
    for sample in samples:
        bearing = math.atan2(
            tower["y"] - sample["y"], tower["x"] - sample["x"]
        )
        errors.append(abs(angle_error(sample["yaw"], bearing)))
    return errors


def segment_intersection(a, b, c, d):
    rx, ry = b[0] - a[0], b[1] - a[1]
    sx, sy = d[0] - c[0], d[1] - c[1]
    denominator = rx * sy - ry * sx
    if abs(denominator) < 1.0e-9:
        return None
    qx, qy = c[0] - a[0], c[1] - a[1]
    t = (qx * sy - qy * sx) / denominator
    u = (qx * ry - qy * rx) / denominator
    if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
        return a[0] + t * rx, a[1] + t * ry
    return None


def detect_gap_crossing(entry, cylinders):
    required = EGO_INFLATION + EGO_TRAJECTORY_CLEARANCE
    best = None
    for first_index, first in enumerate(cylinders):
        for second in cylinders[first_index + 1 :]:
            center_distance = math.hypot(
                second["x"] - first["x"], second["y"] - first["y"]
            )
            physical_gap = center_distance - first["radius"] - second["radius"]
            usable_gap = physical_gap - 2.0 * required
            if usable_gap <= 0.0:
                continue
            for left, right in zip(entry, entry[1:]):
                crossing = segment_intersection(
                    (left["x"], left["y"]),
                    (right["x"], right["y"]),
                    (first["x"], first["y"]),
                    (second["x"], second["y"]),
                )
                if crossing is None:
                    continue
                first_clearance = (
                    math.hypot(crossing[0] - first["x"],
                               crossing[1] - first["y"])
                    - first["radius"]
                )
                second_clearance = (
                    math.hypot(crossing[0] - second["x"],
                               crossing[1] - second["y"])
                    - second["radius"]
                )
                minimum_clearance = min(first_clearance, second_clearance)
                if minimum_clearance + 1.0e-6 < required:
                    continue
                candidate = {
                    "cylinder_gap_crossed": True,
                    "gap_pair": [first["name"], second["name"]],
                    "gap_crossing_xy": [crossing[0], crossing[1]],
                    "physical_surface_gap_m": physical_gap,
                    "usable_gap_after_0_9m_each_side_m": usable_gap,
                    "crossing_minimum_surface_clearance_m": minimum_clearance,
                }
                if best is None or usable_gap < best[
                    "usable_gap_after_0_9m_each_side_m"
                ]:
                    best = candidate
    return best or {
        "cylinder_gap_crossed": False,
        "gap_pair": None,
        "gap_crossing_xy": None,
        "physical_surface_gap_m": None,
        "usable_gap_after_0_9m_each_side_m": None,
        "crossing_minimum_surface_clearance_m": None,
    }


def return_reuse_metrics(entry, returning):
    if not entry or not returning:
        return {
            "return_trace_sample_count": len(returning),
            "return_mean_distance_to_ingress_m": None,
            "return_p95_distance_to_ingress_m": None,
            "return_reuse_ratio_within_1m": None,
            "return_basically_reused_ingress": False,
        }
    distances = [
        min(
            math.hypot(sample["x"] - ingress["x"],
                       sample["y"] - ingress["y"])
            for ingress in entry
        )
        for sample in returning
    ]
    ratio = sum(value <= 1.0 for value in distances) / len(distances)
    return {
        "return_trace_sample_count": len(returning),
        "return_mean_distance_to_ingress_m": sum(distances) / len(distances),
        "return_p95_distance_to_ingress_m": percentile(distances, 0.95),
        "return_reuse_ratio_within_1m": ratio,
        "return_basically_reused_ingress": ratio >= 0.8,
    }


def summarize(samples, world, tower, cylinders):
    entry = [sample for sample in samples
             if sample["state"] == "ENTRY_GATE_TRANSIT"]
    returning = [sample for sample in samples
                 if sample["state"] == "NORMAL_RETURN"]
    horizontal = [
        sample for sample in entry
        if sample["horizontal"] and not sample["vertical"]
    ]
    vertical = [sample for sample in entry if sample["vertical"]]
    orbit = [
        sample for sample in samples
        if sample["state"] in {"TARGET_LOCKED", "NAVIGATING"}
        and sample["target_id"].startswith("l0_s")
    ]
    unique_targets = {}
    for sample in orbit:
        unique_targets[sample["target_id"]] = (
            sample["target_x"], sample["target_y"], sample["target_z"]
        )
    states = {sample["state"] for sample in samples}
    gate_samples = [
        sample for sample in entry
        if sample["target_id"].startswith("ENTRY_GATE")
    ]
    gate = gate_samples[-1] if gate_samples else None
    terminal_return_landed = (
        samples[-1]["state"] == "RETURN_HOME"
        and samples[-1]["z"] <= 0.3
        and ("NORMAL_RETURN" in states or "RETURN_EGRESS" in states)
    )
    completed_orbit_index = max(
        (sample["waypoint"] for sample in samples
         if sample["state"] in {
             "TARGET_LOCKED", "NAVIGATING", "EVALUATING",
             "NORMAL_RETURN", "RETURN_EGRESS"
         }),
        default=-1,
    )
    entry_yaw = tangent_yaw_errors(entry)
    return_yaw = tangent_yaw_errors(returning)
    orbit_yaw = tower_yaw_errors(orbit, tower)
    yaw_rates = []
    for left, right in zip(samples, samples[1:]):
        dt = right["time"] - left["time"]
        if dt > 1.0e-3:
            yaw_rates.append(abs(angle_error(left["yaw"], right["yaw"])) / dt)
    summary = {
        "world_path": str(world.resolve()),
        "world_sha256": hashlib.sha256(world.read_bytes()).hexdigest(),
        "world_cylinders": cylinders,
        "tower_center": [tower["x"], tower["y"]],
        "sample_count": len(samples),
        "duration_s": samples[-1]["time"] - samples[0]["time"],
        "final_state": samples[-1]["state"],
        "actual_total_path_length_m": path_length(samples),
        "ingress_actual_path_length_m": path_length(entry),
        "ingress_planned_reference_length_m": path_length(
            entry, "ref_x", "ref_y", "ref_z"
        ),
        "entry_gate_sector": 8,
        "entry_gate_xyz": (
            [gate["target_x"], gate["target_y"], gate["target_z"]]
            if gate else None
        ),
        "horizontal_path_was_available": bool(horizontal),
        "vertical_escape_was_used": bool(vertical),
        "entry_actual_min_z_m": min((item["z"] for item in entry),
                                    default=None),
        "entry_actual_max_z_m": max((item["z"] for item in entry),
                                    default=None),
        "whole_flight_min_z_m": min(item["z"] for item in samples),
        "whole_flight_max_z_m": max(item["z"] for item in samples),
        "maximum_ingress_replan_count": max(
            (item["replans"] for item in samples), default=0
        ),
        "entry_gate_reached": any(
            item["policy"] == "NOMINAL_3M_ENTRY_AND_ORBIT"
            for item in samples
        ),
        "unique_orbit_target_count": len(unique_targets),
        "orbit_target_max_abs_z_error_m": max(
            (abs(target[2] - NOMINAL_HEIGHT)
             for target in unique_targets.values()),
            default=None,
        ),
        "orbit_target_max_radius_error_m": max(
            (abs(math.hypot(target[0] - tower["x"],
                            target[1] - tower["y"]) - ORBIT_RADIUS)
             for target in unique_targets.values()),
            default=None,
        ),
        "entry_velocity_yaw_mean_abs_error_deg": (
            math.degrees(sum(entry_yaw) / len(entry_yaw))
            if entry_yaw else None
        ),
        "entry_velocity_yaw_p95_abs_error_deg": (
            math.degrees(percentile(entry_yaw, 0.95))
            if entry_yaw else None
        ),
        "orbit_tower_yaw_mean_abs_error_deg": (
            math.degrees(sum(orbit_yaw) / len(orbit_yaw))
            if orbit_yaw else None
        ),
        "return_velocity_yaw_mean_abs_error_deg": (
            math.degrees(sum(return_yaw) / len(return_yaw))
            if return_yaw else None
        ),
        "maximum_observed_yaw_rate_deg_s": (
            math.degrees(max(yaw_rates)) if yaw_rates else None
        ),
        "closed_lap_completed": (
            "GO_TO_EXIT_GATE" in states
            or (completed_orbit_index >= 7 and "NORMAL_RETURN" in states)
        ),
        "return_started": "NORMAL_RETURN" in states,
        "mission_done": "DONE" in states or terminal_return_landed,
        "mission_done_inferred_from_grounded_return": (
            terminal_return_landed and "DONE" not in states
        ),
    }
    summary.update(detect_gap_crossing(entry, cylinders))
    summary.update(return_reuse_metrics(entry, returning))
    return summary


def plot(samples, source, output, tower, cylinders):
    start_time = samples[0]["time"]
    times = [sample["time"] - start_time for sample in samples]
    figure, axes = plt.subplots(
        2, 2, figsize=(16, 12), constrained_layout=True
    )
    trajectory = axes[0][0]
    trajectory.plot(
        [item["x"] for item in samples], [item["y"] for item in samples],
        color="#1f2937", linewidth=1.3, label="Actual trajectory"
    )
    entry = [item for item in samples
             if item["state"] == "ENTRY_GATE_TRANSIT"]
    returning = [item for item in samples if item["state"] == "NORMAL_RETURN"]
    trajectory.plot(
        [item["ref_x"] for item in entry],
        [item["ref_y"] for item in entry],
        color="#f97316", linewidth=1.1, label="EGO ingress reference"
    )
    trajectory.plot(
        [item["x"] for item in returning], [item["y"] for item in returning],
        color="#16a34a", linewidth=1.3, label="Actual reverse return"
    )
    theta = [index * 2.0 * math.pi / 240.0 for index in range(241)]
    trajectory.plot(
        [tower["x"] + ORBIT_RADIUS * math.cos(value) for value in theta],
        [tower["y"] + ORBIT_RADIUS * math.sin(value) for value in theta],
        linestyle=":", color="#2563eb", label="12.5 m orbit"
    )
    trajectory.scatter(
        [item["x"] for item in cylinders],
        [item["y"] for item in cylinders],
        color="#dc2626", s=45, label="Cylinders from current world"
    )
    trajectory.scatter(
        [tower["x"]], [tower["y"]], marker="x", color="#111827", s=70,
        label="Tower centre"
    )
    trajectory.set_aspect("equal", adjustable="box")
    trajectory.set_xlabel("X / East (m)")
    trajectory.set_ylabel("Y / North (m)")
    trajectory.set_title("Live-map ingress, orbit and reverse return")
    trajectory.grid(alpha=0.2)
    trajectory.legend(fontsize=8)

    altitude = axes[0][1]
    altitude.plot(times, [item["z"] for item in samples],
                  color="#111827", label="Actual z")
    altitude.plot(times, [item["ref_z"] for item in samples],
                  color="#f97316", linewidth=1.0, label="EGO reference z")
    altitude.axhline(NOMINAL_HEIGHT, linestyle=":", color="#2563eb",
                     label="Required 3 m")
    altitude.set_xlabel("Mission elapsed time (s)")
    altitude.set_ylabel("Z / Up (m)")
    altitude.set_title("Altitude")
    altitude.grid(alpha=0.2)
    altitude.legend(fontsize=8)

    yaw = axes[1][0]
    yaw.plot(times, [math.degrees(item["yaw"]) for item in samples],
             color="#7c3aed", linewidth=1.0, label="Actual yaw")
    yaw.set_xlabel("Mission elapsed time (s)")
    yaw.set_ylabel("Yaw (deg, wrapped)")
    yaw.set_title("Yaw and mode")
    yaw.grid(alpha=0.2)
    yaw.legend(fontsize=8)

    speed = axes[1][1]
    speed.plot(times, [item["speed"] for item in samples],
               color="#0369a1", linewidth=1.0, label="Horizontal speed")
    tower_mode_times = [
        time for time, item in zip(times, samples)
        if item["yaw_mode"] == "FACE_TOWER"
    ]
    if tower_mode_times:
        speed.axvspan(
            min(tower_mode_times), max(tower_mode_times),
            color="#ddd6fe", alpha=0.45, label="Tower-facing phase"
        )
    speed.set_xlabel("Mission elapsed time (s)")
    speed.set_ylabel("Speed (m/s)")
    speed.set_title("Horizontal speed / tower-facing interval")
    speed.grid(alpha=0.2)
    speed.legend(fontsize=8)

    figure.suptitle(
        "Low-altitude EGO evidence — {}".format(source.name), fontsize=13
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output, dpi=180)
    plt.close(figure)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    source = args.csv.resolve()
    world = args.world.resolve()
    output_dir = args.output_dir.resolve()
    samples = read_samples(source)
    tower, cylinders = read_world(world)
    summary = summarize(samples, world, tower, cylinders)
    output = output_dir / "low_altitude_{}.png".format(source.stem)
    summary_path = output_dir / "low_altitude_{}_summary.json".format(
        source.stem
    )
    plot(samples, source, output, tower, cylinders)
    summary_path.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    shutil.copy2(str(output), str(output_dir / "latest_low_altitude.png"))
    shutil.copy2(
        str(summary_path),
        str(output_dir / "latest_low_altitude_summary.json"),
    )
    print(output)
    print(summary_path)
    print(json.dumps(summary, ensure_ascii=False, sort_keys=True))


if __name__ == "__main__":
    main()
