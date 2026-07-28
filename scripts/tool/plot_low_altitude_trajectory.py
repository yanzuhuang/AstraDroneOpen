#!/usr/bin/env python3
"""Plot and summarize one low-altitude flight against its exact world file."""

import argparse
import csv
import hashlib
import json
import math
import os
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
            "mission_target_id",
            "mission_target_x",
            "mission_target_y",
            "mission_target_z",
            "previous_target_id",
            "target_switch_reason",
            "distance_to_target",
            "ego_goal_publish_count",
            "goal_publication_source",
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
                        "mission_target_id": row["mission_target_id"],
                        "mission_target_x": finite(
                            row, "mission_target_x"
                        ),
                        "mission_target_y": finite(
                            row, "mission_target_y"
                        ),
                        "mission_target_z": finite(
                            row, "mission_target_z"
                        ),
                        "previous_target_id": row["previous_target_id"],
                        "target_switch_reason": row[
                            "target_switch_reason"
                        ],
                        "distance_to_target": finite(
                            row, "distance_to_target"
                        ),
                        "goal_publish_count": int(
                            row["ego_goal_publish_count"]
                        ),
                        "goal_source": row["goal_publication_source"],
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
    if max(sample["speed"] for sample in samples) <= 1.0e-6:
        # Some MAVROS/PX4 SITL combinations publish a valid pose while the
        # recorded horizontal twist remains exactly zero. Recover an evidence
        # speed from a centred position difference instead of drawing a
        # misleading zero-speed flight.
        for index, sample in enumerate(samples):
            left = samples[max(0, index - 2)]
            right = samples[min(len(samples) - 1, index + 2)]
            dt = right["time"] - left["time"]
            if dt > 1.0e-3:
                sample["speed"] = math.hypot(
                    right["x"] - left["x"], right["y"] - left["y"]
                ) / dt
    return samples


def mission_goal_events(samples):
    events = []
    seen = set()
    for sample in samples:
        count = sample["goal_publish_count"]
        if count <= 0 or count in seen:
            continue
        seen.add(count)
        events.append(
            {
                "count": count,
                "id": sample["mission_target_id"],
                "x": sample["mission_target_x"],
                "y": sample["mission_target_y"],
                "z": sample["mission_target_z"],
                "previous": sample["previous_target_id"],
                "reason": sample["target_switch_reason"],
                "distance": sample["distance_to_target"],
                "source": sample["goal_source"],
            }
        )
    return sorted(events, key=lambda item: item["count"])


def read_bag_visuals(path):
    visuals = {
        "fixed_route": [],
        "global_segments": [],
        "local_control_segments": [],
        "topics": set(),
    }
    if path is None:
        return visuals
    try:
        import rosbag
    except ImportError as error:
        raise RuntimeError(
            "rosbag Python module is required when --bag is used"
        ) from error

    with rosbag.Bag(str(path), "r") as bag:
        for topic, message, _stamp in bag.read_messages(
            topics=[
                "/tower_mission/mission_route",
                "/ego_planner_node/global_list",
                "/ego_planner_node/optimal_list",
                "/planning/goal_velocity",
                "/planning/goal_path_hint",
            ]
        ):
            visuals["topics"].add(topic)
            if topic == "/tower_mission/mission_route":
                visuals["fixed_route"] = [
                    (pose.pose.position.x,
                     pose.pose.position.y,
                     pose.pose.position.z)
                    for pose in message.poses
                ]
                continue
            # displayMarkerList publishes a SPHERE_LIST with id=0 and the
            # corresponding LINE_STRIP with id=1000. Keep only the line.
            if getattr(message, "type", None) != 4 or not message.points:
                continue
            segment = [
                (point.x, point.y, point.z) for point in message.points
            ]
            if topic == "/ego_planner_node/global_list":
                visuals["global_segments"].append(segment)
            elif topic == "/ego_planner_node/optimal_list":
                visuals["local_control_segments"].append(segment)
    return visuals


def pose_values(element):
    if element is None or not element.text:
        return None
    values = [float(value) for value in element.text.split()]
    return values if len(values) >= 3 else None


def mesh_path(uri, world_path):
    if uri.startswith("file://"):
        candidate = Path(uri[7:])
        return candidate if candidate.is_file() else None
    if not uri.startswith("model://"):
        candidate = (world_path.parent / uri).resolve()
        return candidate if candidate.is_file() else None
    relative = Path(uri[len("model://"):])
    roots = [
        world_path.parent.parent / "astra_gazebo_models",
        Path("/usr/share/gazebo-11/models"),
        Path.home() / ".gazebo/models",
    ]
    roots.extend(
        Path(value)
        for value in os.environ.get("GAZEBO_MODEL_PATH", "").split(":")
        if value
    )
    for root in roots:
        candidate = root / relative
        if candidate.is_file():
            return candidate
    return None


def collada_bounds(path):
    root = ET.parse(str(path)).getroot()
    unit = root.find(".//{*}asset/{*}unit")
    meter = float(unit.get("meter", "1")) if unit is not None else 1.0
    transform_scale = 0.0
    for matrix in root.findall(".//{*}visual_scene/{*}node/{*}matrix"):
        if not matrix.text:
            continue
        values = [float(value) for value in matrix.text.split()]
        if len(values) != 16:
            continue
        transform_scale = max(
            transform_scale,
            math.sqrt(values[0] ** 2 + values[1] ** 2 + values[2] ** 2),
            math.sqrt(values[4] ** 2 + values[5] ** 2 + values[6] ** 2),
            math.sqrt(values[8] ** 2 + values[9] ** 2 + values[10] ** 2),
        )
    if transform_scale <= 0.0:
        transform_scale = 1.0
    points = []
    for source in root.findall(".//{*}source"):
        if "position" not in source.get("id", "").lower():
            continue
        array = source.find("{*}float_array")
        if array is None or not array.text:
            continue
        values = [float(value) for value in array.text.split()]
        points.extend(zip(values[0::3], values[1::3], values[2::3]))
    if not points:
        return None
    factor = meter * transform_scale
    return (
        min(point[0] for point in points) * factor,
        max(point[0] for point in points) * factor,
        min(point[1] for point in points) * factor,
        max(point[1] for point in points) * factor,
        min(point[2] for point in points) * factor,
        max(point[2] for point in points) * factor,
    )


def obj_bounds(path):
    points = []
    with path.open(encoding="utf-8", errors="ignore") as stream:
        for line in stream:
            if line.startswith("v "):
                values = line.split()
                if len(values) >= 4:
                    points.append(tuple(float(value) for value in values[1:4]))
    if not points:
        return None
    return (
        min(point[0] for point in points),
        max(point[0] for point in points),
        min(point[1] for point in points),
        max(point[1] for point in points),
        min(point[2] for point in points),
        max(point[2] for point in points),
    )


def collision_bounds(collision, world_path):
    geometry = collision.find("geometry")
    if geometry is None:
        return None
    radius = geometry.findtext("cylinder/radius")
    length = geometry.findtext("cylinder/length")
    if radius is not None and length is not None:
        radius = float(radius)
        half_length = 0.5 * float(length)
        return (-radius, radius, -radius, radius, -half_length, half_length)
    size = geometry.findtext("box/size")
    if size:
        values = [0.5 * float(value) for value in size.split()]
        if len(values) >= 3:
            return (-values[0], values[0], -values[1], values[1],
                    -values[2], values[2])
    sphere = geometry.findtext("sphere/radius")
    if sphere is not None:
        radius = float(sphere)
        return (-radius, radius, -radius, radius, -radius, radius)
    uri = geometry.findtext("mesh/uri")
    if not uri:
        return None
    path = mesh_path(uri, world_path)
    if path is None:
        return None
    if path.suffix.lower() == ".dae":
        bounds = collada_bounds(path)
    elif path.suffix.lower() == ".obj":
        bounds = obj_bounds(path)
    else:
        bounds = None
    scale = geometry.findtext("mesh/scale")
    if bounds is None or not scale:
        return bounds
    values = [float(value) for value in scale.split()]
    if len(values) < 3:
        return bounds
    return (
        bounds[0] * values[0], bounds[1] * values[0],
        bounds[2] * values[1], bounds[3] * values[1],
        bounds[4] * values[2], bounds[5] * values[2],
    )


def model_footprint(model, pose, world_path):
    combined = None
    for collision in model.findall(".//collision"):
        bounds = collision_bounds(collision, world_path)
        if bounds is None:
            continue
        local_pose = pose_values(collision.find("pose")) or [0.0, 0.0, 0.0]
        shifted = (
            bounds[0] + local_pose[0], bounds[1] + local_pose[0],
            bounds[2] + local_pose[1], bounds[3] + local_pose[1],
            bounds[4] + local_pose[2], bounds[5] + local_pose[2],
        )
        if combined is None:
            combined = shifted
        else:
            combined = (
                min(combined[0], shifted[0]), max(combined[1], shifted[1]),
                min(combined[2], shifted[2]), max(combined[3], shifted[3]),
                min(combined[4], shifted[4]), max(combined[5], shifted[5]),
            )
    if combined is None:
        return None
    yaw = pose[5] if len(pose) > 5 else 0.0
    corners = [
        (x, y)
        for x in (combined[0], combined[1])
        for y in (combined[2], combined[3])
    ]
    rotated = [
        (math.cos(yaw) * x - math.sin(yaw) * y,
         math.sin(yaw) * x + math.cos(yaw) * y)
        for x, y in corners
    ]
    width = max(x for x, _ in rotated) - min(x for x, _ in rotated)
    depth = max(y for _, y in rotated) - min(y for _, y in rotated)
    return {
        "radius": 0.5 * max(width, depth),
        "width": width,
        "depth": depth,
        "height": combined[5] - combined[4],
        "minimum_z": pose[2] + combined[4],
        "maximum_z": pose[2] + combined[5],
    }


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
        footprint = model_footprint(model, values, path)
        if footprint is not None:
            item.update(footprint)
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
    obstacles = [
        item
        for name, item in sorted(models.items())
        if name != "radio_tower"
        and "radius" in item
        and item["minimum_z"] <= NOMINAL_HEIGHT
        and item["maximum_z"] >= NOMINAL_HEIGHT
    ]
    if not obstacles:
        raise ValueError("no collision obstacle intersects nominal flight height")
    return tower, obstacles


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


def detect_gap_crossing(entry, obstacles):
    required = EGO_INFLATION + EGO_TRAJECTORY_CLEARANCE
    best = None
    relevant = [
        obstacle for obstacle in obstacles
        if any(
            math.hypot(sample["x"] - obstacle["x"],
                       sample["y"] - obstacle["y"])
            <= obstacle["radius"] + 15.0
            for sample in entry
        )
    ]
    for first_index, first in enumerate(relevant):
        for second in relevant[first_index + 1 :]:
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
                    "obstacle_gap_crossed": True,
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
        "obstacle_gap_crossed": False,
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


def summarize(samples, world, tower, obstacles, visuals):
    goal_events = mission_goal_events(samples)
    goal_order = [item["id"] for item in goal_events]
    expected_goal_order = [
        "ENTRY_GATE",
        "WP1", "WP2", "WP3", "WP4", "WP5", "WP6", "WP7", "WP8",
        "WP1",
        "EXIT_GATE",
        "HOME_HOVER",
    ]
    temporary_prefixes = (
        "LIVE_", "VERTICAL_", "EXECUTED_INGRESS_", "INGRESS_REVERSE_",
    )
    temporary_goals = [
        item for item in goal_events
        if item["id"].startswith(temporary_prefixes)
    ]
    global_segments = visuals["global_segments"]
    endpoint_errors = []
    for segment, event in zip(global_segments, goal_events):
        if not segment:
            continue
        endpoint = segment[-1]
        endpoint_errors.append(
            math.sqrt(
                (endpoint[0] - event["x"]) ** 2
                + (endpoint[1] - event["y"]) ** 2
                + (endpoint[2] - event["z"]) ** 2
            )
        )
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
        "world_obstacles": obstacles,
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
        "entry_gate_reached": bool(orbit),
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
        "closed_lap_completed": "GO_TO_EXIT_GATE" in states,
        "return_started": "NORMAL_RETURN" in states,
        "mission_done": "DONE" in states or terminal_return_landed,
        "mission_done_inferred_from_grounded_return": (
            terminal_return_landed and "DONE" not in states
        ),
        "formal_goal_publish_count": len(goal_events),
        "formal_goal_sequence": goal_order,
        "expected_formal_goal_sequence": expected_goal_order,
        "formal_goal_sequence_valid": goal_order == expected_goal_order,
        "temporary_mission_goal_count": len(temporary_goals),
        "temporary_mission_goal_ids": [
            item["id"] for item in temporary_goals
        ],
        "goal_path_hint_message_present": (
            "/planning/goal_path_hint" in visuals["topics"]
        ),
        "goal_velocity_message_present": (
            "/planning/goal_velocity" in visuals["topics"]
        ),
        "fixed_mission_route_point_count": len(visuals["fixed_route"]),
        "ego_global_trajectory_segment_count": len(global_segments),
        "ego_local_control_segment_count": len(
            visuals["local_control_segments"]
        ),
        "ego_global_endpoint_max_error_m": (
            max(endpoint_errors) if endpoint_errors else None
        ),
        "global_trajectory_matches_formal_targets": (
            len(global_segments) >= len(goal_events)
            and bool(endpoint_errors)
            and max(endpoint_errors) <= 0.5
        ),
    }
    summary.update(detect_gap_crossing(entry, obstacles))
    summary.update(return_reuse_metrics(entry, returning))
    return summary


def plot(samples, source, output, tower, obstacles, visuals):
    start_time = samples[0]["time"]
    times = [sample["time"] - start_time for sample in samples]
    figure, axes = plt.subplots(
        2, 2, figsize=(16, 12), constrained_layout=True
    )
    trajectory = axes[0][0]
    fixed_route = visuals["fixed_route"]
    if not fixed_route:
        fixed_route = [
            (item["x"], item["y"], item["z"])
            for item in mission_goal_events(samples)
        ]
    if fixed_route:
        trajectory.plot(
            [item[0] for item in fixed_route],
            [item[1] for item in fixed_route],
            color="#2563eb", linewidth=1.6, linestyle="--",
            label="Fixed mission waypoint line",
        )
    for index, segment in enumerate(visuals["global_segments"]):
        trajectory.plot(
            [item[0] for item in segment],
            [item[1] for item in segment],
            color="#0f766e", linewidth=1.0, alpha=0.75,
            label="EGO global trajectory" if index == 0 else None,
        )
    local_groups = {}
    for item in samples:
        local_groups.setdefault(item["goal_publish_count"], []).append(item)
    for index, group in enumerate(
            value for key, value in sorted(local_groups.items()) if key > 0):
        trajectory.plot(
            [item["ref_x"] for item in group],
            [item["ref_y"] for item in group],
            color="#f97316", linewidth=0.9, alpha=0.70,
            label="EGO local trajectory (PositionCommand)"
            if index == 0 else None,
        )
    trajectory.plot(
        [item["x"] for item in samples], [item["y"] for item in samples],
        color="#1f2937", linewidth=1.3, label="Actual trajectory"
    )
    entry = [item for item in samples
             if item["state"] == "ENTRY_GATE_TRANSIT"]
    returning = [item for item in samples if item["state"] == "NORMAL_RETURN"]
    theta = [index * 2.0 * math.pi / 240.0 for index in range(241)]
    trajectory.plot(
        [tower["x"] + ORBIT_RADIUS * math.cos(value) for value in theta],
        [tower["y"] + ORBIT_RADIUS * math.sin(value) for value in theta],
        linestyle=":", color="#2563eb", label="12.5 m orbit"
    )
    minimum_x = min(item["x"] for item in samples) - 15.0
    maximum_x = max(item["x"] for item in samples) + 15.0
    minimum_y = min(item["y"] for item in samples) - 15.0
    maximum_y = max(item["y"] for item in samples) + 15.0
    visible_obstacles = [
        item for item in obstacles
        if minimum_x <= item["x"] <= maximum_x
        and minimum_y <= item["y"] <= maximum_y
    ]
    trajectory.scatter(
        [item["x"] for item in visible_obstacles],
        [item["y"] for item in visible_obstacles],
        color="#dc2626", s=45, label="Obstacles from current world"
    )
    trajectory.scatter(
        [tower["x"]], [tower["y"]], marker="x", color="#111827", s=70,
        label="Tower centre"
    )
    trajectory.set_aspect("equal", adjustable="box")
    trajectory.set_xlabel("X / East (m)")
    trajectory.set_ylabel("Y / North (m)")
    trajectory.set_title(
        "Fixed route vs EGO global/local vs actual trajectory"
    )
    trajectory.grid(alpha=0.2)
    trajectory.legend(fontsize=8)

    altitude = axes[0][1]
    altitude.plot(times, [item["z"] for item in samples],
                  color="#111827", label="Actual z")
    altitude.plot(times, [item["ref_z"] for item in samples],
                  color="#f97316", linewidth=1.0,
                  label="EGO local PositionCommand z")
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
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--world", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    source = args.csv.resolve()
    world = args.world.resolve()
    output_dir = args.output_dir.resolve()
    samples = read_samples(source)
    visuals = read_bag_visuals(
        args.bag.resolve() if args.bag is not None else None
    )
    tower, obstacles = read_world(world)
    summary = summarize(samples, world, tower, obstacles, visuals)
    output = output_dir / "low_altitude_{}.png".format(source.stem)
    summary_path = output_dir / "low_altitude_{}_summary.json".format(
        source.stem
    )
    plot(samples, source, output, tower, obstacles, visuals)
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
