#!/usr/bin/env python3
"""Audit one Stage 3 ROS1 flight bag and emit machine-readable JSON."""

import argparse
import json
import math
import re
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np
import rosbag


REPO_ROOT = Path(__file__).resolve().parents[2]
RUNTIME_ARTIFACTS_ROOT = REPO_ROOT / "runtime_artifacts"


def require_runtime_artifact_path(path):
    resolved = path.resolve()
    try:
        resolved.relative_to(RUNTIME_ARTIFACTS_ROOT)
    except ValueError:
        raise ValueError(
            "Output path must be under {}: {}".format(
                RUNTIME_ARTIFACTS_ROOT, resolved
            )
        )
    return resolved


CONTROL_TOPICS = (
    "/mavros/setpoint_raw/local",
    "/mavros/setpoint_position/local",
    "/mavros/setpoint_velocity/cmd_vel",
    "/mavros/setpoint_attitude/attitude",
    "/mavros/setpoint_attitude/thrust",
)

REQUIRED_TOPICS = (
    "/clock",
    "/livox/lidar",
    "/cloud_registered",
    "/stage3/cloud_registered_filtered",
    "/Odometry",
    "/mavros/local_position/odom",
    "/mavros/local_position/pose",
    "/mavros/state",
    "/mavros/extended_state",
    "/mavros/setpoint_raw/local",
    "/grid_map/occupancy_inflate",
    "/stage3/occupancy_inflate",
    "/planning/goal",
    "/planning/bspline",
    "/planning/pos_cmd",
    "/planner/status",
    "/ego_mavros_bridge/state",
    "/ego_mavros_bridge/tracking_error",
    "/tower_mission/state",
    "/tower_mission/current_target",
    "/tower_mission/current_sector",
    "/tower_mission/candidate_targets",
    "/tower_mission/face_tower",
    "/tower_mission/selected_tower_center",
    "/tf",
    "/tf_static",
    "/rosout",
)

READ_TOPICS = (
    "/Odometry",
    "/stage3/cloud_registered_filtered",
    "/stage3/occupancy_inflate",
    "/planning/goal",
    "/planning/pos_cmd",
    "/planning/cancel",
    "/planner/status",
    "/ego_mavros_bridge/state",
    "/ego_mavros_bridge/tracking_error",
    "/tower_mission/state",
    "/tower_mission/current_sector",
    "/tower_mission/face_tower",
    "/tower_mission/selected_tower_center",
    "/mavros/local_position/odom",
    "/mavros/local_position/pose",
    "/mavros/state",
    "/mavros/extended_state",
    "/mavros/setpoint_raw/local",
    "/rosout",
)


def percentile(values, percentage):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentage / 100.0
    low = int(math.floor(index))
    high = int(math.ceil(index))
    if low == high:
        return ordered[low]
    fraction = index - low
    return ordered[low] * (1.0 - fraction) + ordered[high] * fraction


def transition_event(timestamp, value):
    return {"time": round(timestamp, 3), "state": value}


def pointcloud_xyz(message):
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    # Stage 3 occupancy uses FLOAT32 x/y/z. Reject an unexpected layout rather
    # than silently interpreting a different PointField datatype.
    if any(fields[name].datatype != 7 for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float32)
    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [
                fields["x"].offset,
                fields["y"].offset,
                fields["z"].offset,
            ],
            "itemsize": message.point_step,
        }
    )
    count = message.width * message.height
    array = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack((array["x"], array["y"], array["z"]))
    return points[np.isfinite(points).all(axis=1)]


def audit(
    path,
    occupancy_stride,
    tracking_error_limit,
    tracking_error_duration,
    expected_entry_sector,
):
    tracking_errors = []
    tracking_above_limit_count = 0
    tracking_above_limit_since = None
    tracking_last_stamp = None
    tracking_max_continuous_duration = 0.0
    speeds = []
    raw_masks = Counter()
    planner_states = Counter()
    planner_reasons = Counter()
    planner_max_failures = 0
    planner_flag_counts = Counter()
    frames = defaultdict(set)
    mission_states = []
    bridge_states = []
    current_sector_events = []
    sector_targets = []
    entry_gates = []
    ascent_targets = []
    layer_transition_targets = []
    exit_gate_targets = []
    normal_return_targets = []
    return_egress_targets = []
    ascent_odom_positions = []
    transition_odom_positions = []
    exit_gate_odom_positions = []
    return_home_odom_positions = []
    return_home_planning_goals = []
    tower_yaw_errors = []
    tower_center = None
    face_tower_active = False
    current_mission_state = ""
    cancel_count = 0
    final_mavros = None
    final_extended = None
    latest_pose = None
    occupancy_samples = 0
    occupancy_min_distance = math.inf
    occupancy_min_time = None
    occupancy_index = 0
    current_sector_index = 0

    with rosbag.Bag(path, "r") as bag:
        info = bag.get_type_and_topic_info()
        topic_counts = {
            topic: topic_info.message_count
            for topic, topic_info in info.topics.items()
        }
        connection_publishers = defaultdict(set)
        for connection in bag._get_connections():
            caller = connection.header.get("callerid", "unknown")
            if isinstance(caller, bytes):
                caller = caller.decode("utf-8", errors="replace")
            connection_publishers[connection.topic].add(caller)

        start = bag.get_start_time()
        end = bag.get_end_time()
        for topic, message, stamp in bag.read_messages(topics=READ_TOPICS):
            timestamp = stamp.to_sec()
            if topic == "/ego_mavros_bridge/tracking_error":
                if math.isfinite(message.data):
                    tracking_error = float(message.data)
                    tracking_errors.append(tracking_error)
                    if tracking_error > tracking_error_limit:
                        tracking_above_limit_count += 1
                        if (
                            tracking_above_limit_since is None
                            or tracking_last_stamp is None
                            or timestamp - tracking_last_stamp > 0.2
                        ):
                            tracking_above_limit_since = timestamp
                        tracking_max_continuous_duration = max(
                            tracking_max_continuous_duration,
                            timestamp - tracking_above_limit_since,
                        )
                    else:
                        tracking_above_limit_since = None
                    tracking_last_stamp = timestamp
            elif topic == "/ego_mavros_bridge/state":
                event = transition_event(timestamp, message.data)
                if latest_pose is not None:
                    event["mavros_position"] = [
                        float(latest_pose[0]),
                        float(latest_pose[1]),
                        float(latest_pose[2]),
                    ]
                bridge_states.append(event)
            elif topic == "/tower_mission/state":
                current_mission_state = message.data
                mission_states.append(transition_event(timestamp, message.data))
            elif topic == "/tower_mission/current_sector":
                current_sector_index = int(message.data)
                current_sector_events.append(
                    {"time": round(timestamp, 3), "index": current_sector_index}
                )
            elif topic == "/tower_mission/face_tower":
                face_tower_active = bool(message.data)
            elif topic == "/tower_mission/selected_tower_center":
                tower_center = np.array(
                    [message.point.x, message.point.y], dtype=np.float64
                )
            elif topic == "/planning/cancel":
                cancel_count += 1
            elif topic == "/planner/status":
                planner_states[message.planner_state] += 1
                planner_reasons[message.failure_reason] += 1
                planner_max_failures = max(
                    planner_max_failures, int(message.consecutive_plan_failures)
                )
                if message.goal_in_collision:
                    planner_flag_counts["goal_in_collision"] += 1
                if message.current_position_in_collision:
                    planner_flag_counts["current_position_in_collision"] += 1
                if message.emergency_stop_active:
                    planner_flag_counts["emergency_stop_active"] += 1
            elif topic == "/mavros/setpoint_raw/local":
                raw_masks[int(message.type_mask)] += 1
            elif topic == "/mavros/state":
                final_mavros = {
                    "time": round(timestamp, 3),
                    "connected": bool(message.connected),
                    "armed": bool(message.armed),
                    "mode": message.mode,
                }
            elif topic == "/mavros/extended_state":
                final_extended = {
                    "time": round(timestamp, 3),
                    "landed_state": int(message.landed_state),
                }
            elif topic == "/mavros/local_position/odom":
                velocity = message.twist.twist.linear
                speed = math.sqrt(
                    velocity.x * velocity.x
                    + velocity.y * velocity.y
                    + velocity.z * velocity.z
                )
                if math.isfinite(speed):
                    speeds.append(speed)
            elif topic == "/mavros/local_position/pose":
                position = message.pose.position
                latest_pose = np.array(
                    [position.x, position.y, position.z], dtype=np.float64
                )
            elif topic == "/Odometry":
                frames[topic].add(message.header.frame_id)
                position = message.pose.pose.position
                odom_position = np.array(
                    [position.x, position.y, position.z], dtype=np.float64
                )
                if current_mission_state == "SEGMENTED_CLIMB":
                    ascent_odom_positions.append(odom_position)
                elif current_mission_state == "LAYER_TRANSITION":
                    transition_odom_positions.append(odom_position)
                elif current_mission_state == "GO_TO_EXIT_GATE":
                    exit_gate_odom_positions.append(odom_position)
                elif current_mission_state == "RETURN_HOME":
                    return_home_odom_positions.append(odom_position)
                if (
                    face_tower_active
                    and tower_center is not None
                    and current_mission_state
                    in {
                        "ENTRY_GATE_TRANSIT",
                        "TARGET_LOCKED",
                        "NAVIGATING",
                        "EVALUATING",
                        "LAYER_TRANSITION",
                    }
                ):
                    orientation = message.pose.pose.orientation
                    yaw = math.atan2(
                        2.0
                        * (
                            orientation.w * orientation.z
                            + orientation.x * orientation.y
                        ),
                        1.0
                        - 2.0
                        * (
                            orientation.y * orientation.y
                            + orientation.z * orientation.z
                        ),
                    )
                    expected = math.atan2(
                        tower_center[1] - position.y,
                        tower_center[0] - position.x,
                    )
                    error = abs(
                        math.atan2(
                            math.sin(yaw - expected),
                            math.cos(yaw - expected),
                        )
                    )
                    if math.isfinite(error):
                        tower_yaw_errors.append(error)
            elif topic == "/stage3/occupancy_inflate":
                frames[topic].add(message.header.frame_id)
                occupancy_index += 1
                if (
                    latest_pose is not None
                    and occupancy_index % occupancy_stride == 0
                ):
                    points = pointcloud_xyz(message)
                    if points.size:
                        distances = np.linalg.norm(points - latest_pose, axis=1)
                        distance = float(np.min(distances))
                        occupancy_samples += 1
                        if distance < occupancy_min_distance:
                            occupancy_min_distance = distance
                            occupancy_min_time = timestamp
            elif topic in (
                "/stage3/cloud_registered_filtered",
                "/planning/goal",
                "/planning/pos_cmd",
            ):
                frames[topic].add(message.header.frame_id)
                if (
                    topic == "/planning/goal"
                    and current_mission_state == "RETURN_HOME"
                ):
                    position = message.pose.position
                    return_home_planning_goals.append(
                        {
                            "time": round(timestamp, 3),
                            "position": [
                                float(position.x),
                                float(position.y),
                                float(position.z),
                            ],
                        }
                    )
            elif topic == "/rosout":
                text = message.msg
                gate = re.search(
                    r"(?:ENTRY_GATE selected: layer=\d+ |"
                    r"ENTRY_GATE selected: |"
                    r"ENTRY_GATE locked only after safe-altitude map dwell: |"
                    r"fixed ENTRY_GATE locked after safe-altitude map dwell: |"
                    r"dynamic ENTRY_GATE/EXIT_GATE locked: layer=\d+ )"
                    r"(ENTRY_GATE_(?:a\d+|s\d+(?:_c\d+)?))",
                    text,
                )
                if gate:
                    entry_gates.append(
                        {"time": round(timestamp, 3), "id": gate.group(1)}
                    )
                initial = re.search(
                    r"initial safe sector selected: sector_id=(\d+), "
                    r"target=(s\d+_c\d+)",
                    text,
                )
                first_selected = re.search(
                    r"FIRST_WAYPOINT selected waypoint=\d+ "
                    r"internal_index=(\d+).* target="
                    r"((?:l\d+_)?s\d+_c\d+)",
                    text,
                )
                closing_anchor = re.search(
                    r"closing layer=(\d+) lap=\d+ at start anchor "
                    r"waypoint=\d+.* target=((?:l\d+_)?s\d+_c\d+)",
                    text,
                )
                transition_anchor = re.search(
                    r"next-layer first waypoint confirmed by actual arrival: "
                    r"layer=(\d+).* target=((?:l\d+_)?s\d+_c\d+)",
                    text,
                )
                locked = re.search(
                    r"safe candidate locked: ((?:l\d+_)?s\d+_c\d+)", text
                )
                fixed = re.search(
                    r"fixed first waypoint locked: layer=(\d+) "
                    r"sector_id=(\d+) target=((?:l\d+_)?s\d+_c\d+)",
                    text,
                )
                goal = re.search(
                    r"goal phase=\S+ layer=(\d+) sector=(-?\d+) "
                    r"id=(\S+) xyz=\(([-+0-9.]+), ([-+0-9.]+), "
                    r"([-+0-9.]+)\)",
                    text,
                )
                if transition_anchor:
                    target_id = transition_anchor.group(2)
                    match = re.search(r"(?:l(\d+)_)?s(\d+)_c\d+", target_id)
                    sector_targets.append(
                        {
                            "time": round(timestamp, 3),
                            "layer_id": int(transition_anchor.group(1)),
                            "sector_id": int(match.group(2)),
                            "target_id": target_id,
                            "visit_index": current_sector_index,
                        }
                    )
                elif closing_anchor:
                    target_id = closing_anchor.group(2)
                    match = re.search(r"(?:l(\d+)_)?s(\d+)_c\d+", target_id)
                    sector_targets.append(
                        {
                            "time": round(timestamp, 3),
                            "layer_id": int(closing_anchor.group(1)),
                            "sector_id": int(match.group(2)),
                            "target_id": target_id,
                            "visit_index": current_sector_index,
                        }
                    )
                elif first_selected:
                    target_id = first_selected.group(2)
                    match = re.search(r"(?:l(\d+)_)?s(\d+)_c\d+", target_id)
                    sector_targets.append(
                        {
                            "time": round(timestamp, 3),
                            "layer_id": (
                                int(match.group(1))
                                if match.group(1) is not None
                                else 0
                            ),
                            "sector_id": int(first_selected.group(1)),
                            "target_id": target_id,
                            "visit_index": current_sector_index,
                        }
                    )
                elif fixed:
                    sector_targets.append(
                        {
                            "time": round(timestamp, 3),
                            "layer_id": int(fixed.group(1)),
                            "sector_id": int(fixed.group(2)),
                            "target_id": fixed.group(3),
                            "visit_index": current_sector_index,
                        }
                    )
                elif initial:
                    sector_targets.append(
                        {
                            "time": round(timestamp, 3),
                            "layer_id": 0,
                            "sector_id": int(initial.group(1)),
                            "target_id": initial.group(2),
                            "visit_index": current_sector_index,
                        }
                    )
                elif locked:
                    target_id = locked.group(1)
                    match = re.search(
                        r"(?:l(\d+)_)?s(\d+)_c\d+", target_id
                    )
                    sector_targets.append(
                        {
                            "time": round(timestamp, 3),
                            "layer_id": (
                                int(match.group(1))
                                if match and match.group(1) is not None
                                else 0
                            ),
                            "sector_id": int(match.group(2)),
                            "target_id": target_id,
                            "visit_index": current_sector_index,
                        }
                    )
                if goal:
                    event = {
                        "time": round(timestamp, 3),
                        "layer_id": int(goal.group(1)),
                        "sector_id": int(goal.group(2)),
                        "target_id": goal.group(3),
                        "position": [
                            float(goal.group(4)),
                            float(goal.group(5)),
                            float(goal.group(6)),
                        ],
                    }
                    if event["target_id"].startswith("INGRESS_REVERSE_"):
                        normal_return_targets.append(event)
                    elif event["target_id"].startswith(
                        "ASCENT_CHANNEL_"
                    ) and "VERTICAL_ASCENT" in event["target_id"]:
                        ascent_targets.append(event)
                    elif event["target_id"].startswith("LAYER_TRANSITION_"):
                        layer_transition_targets.append(event)
                    elif event["target_id"].startswith("EXIT_GATE_"):
                        exit_gate_targets.append(event)
                        normal_return_targets.append(event)
                    elif event["target_id"].startswith("RETURN_"):
                        return_egress_targets.append(event)

    sector_visits = []
    for target in sector_targets:
        if (
            not sector_visits
            or sector_visits[-1]["target_id"] != target["target_id"]
        ):
            sector_visits.append(target)
    unique_sector_ids = []
    for target in sector_visits:
        if target["sector_id"] not in unique_sector_ids:
            unique_sector_ids.append(target["sector_id"])
    repeated_sector_ids = [
        sector_id
        for sector_id, count in Counter(
            item["sector_id"] for item in sector_visits
        ).items()
        if count > 1
    ]
    visits_by_layer = defaultdict(list)
    for item in sector_visits:
        visits_by_layer[item.get("layer_id", 0)].append(item)
    closed_layers = []
    for layer_id, visits in sorted(visits_by_layer.items()):
        ids = [item["sector_id"] for item in visits]
        if (
            len(ids) == 9
            and ids[0] == ids[-1]
            and set(ids[:-1]) == set(range(8))
            and all(
                next_sector == (sector + 1) % 8
                for sector, next_sector in zip(ids, ids[1:])
            )
        ):
            closed_layers.append(layer_id)
    closed_lap_count = len(closed_layers)
    closed_lap_valid = closed_layers == [0, 1]

    def unique_goal_positions(events):
        unique = []
        for event in events:
            if not unique or unique[-1]["target_id"] != event["target_id"]:
                unique.append(event)
        return unique

    ascent_targets = unique_goal_positions(ascent_targets)
    layer_transition_targets = unique_goal_positions(
        layer_transition_targets
    )
    exit_gate_targets = unique_goal_positions(exit_gate_targets)
    normal_return_targets = unique_goal_positions(normal_return_targets)
    ascent_heights = [round(item["position"][2], 2) for item in ascent_targets]
    ascent_xy = {
        (round(item["position"][0], 2), round(item["position"][1], 2))
        for item in ascent_targets
    }
    vertical_ascent_valid = (
        ascent_heights == [10.0, 18.0, 26.0] and len(ascent_xy) == 1
    )
    transition_heights = [
        round(item["position"][2], 2) for item in layer_transition_targets
    ]
    transition_xy = {
        (round(item["position"][0], 2), round(item["position"][1], 2))
        for item in layer_transition_targets
    }
    transition_command_same_xy = len(transition_xy) == 1
    layer_transition_valid = (
        bool(transition_heights)
        and transition_heights[-1] == 22.0
        and all(
            next_height <= height
            for height, next_height in zip(
                transition_heights, transition_heights[1:]
            )
        )
    )
    dynamic_lower_exit_gate_valid = (
        len(exit_gate_targets) == 1
        and exit_gate_targets[0]["layer_id"] == 1
        and round(exit_gate_targets[0]["position"][2], 2) == 22.0
        and exit_gate_targets[0]["target_id"].startswith("EXIT_GATE_L1_Z22")
    )
    ascent_xy_error = None
    if ascent_targets and ascent_odom_positions:
        reference = np.array(ascent_targets[0]["position"][:2])
        ascent_xy_error = max(
            float(np.linalg.norm(position[:2] - reference))
            for position in ascent_odom_positions
        )
    transition_xy_error = None
    if layer_transition_targets and transition_odom_positions:
        reference = np.array(layer_transition_targets[0]["position"][:2])
        transition_xy_error = max(
            float(np.linalg.norm(position[:2] - reference))
            for position in transition_odom_positions
        )
    return_target_ids = [item["target_id"] for item in return_egress_targets]
    return_gate = next(
        (
            item
            for item in return_egress_targets
            if item["target_id"].startswith("RETURN_GATE_")
        ),
        None,
    )
    return_overhead = next(
        (
            item
            for item in return_egress_targets
            if item["target_id"] == "RETURN_HOME_OVERHEAD"
        ),
        None,
    )
    return_home_radial_valid = (
        any("_RADIAL" in target_id for target_id in return_target_ids)
        and any("_ARC_" in target_id for target_id in return_target_ids)
        and return_gate is not None
        and return_overhead is not None
        and math.hypot(
            return_gate["position"][0] - return_overhead["position"][0],
            return_gate["position"][1] - return_overhead["position"][1],
        )
        < 3.0
    )
    mission_state_names = [event["state"] for event in mission_states]
    bridge_state_names = [event["state"] for event in bridge_states]
    return_egress_selected = "RETURN_EGRESS" in mission_state_names
    direct_normal_return_selected = (
        "GO_TO_EXIT_GATE" in mission_state_names
        and "RETURN_HOME" in mission_state_names
        and dynamic_lower_exit_gate_valid
        and "NORMAL_RETURN" not in mission_state_names
        and not return_egress_selected
    )
    home_hover_before_landing = (
        "HOME_HOVER" in bridge_state_names
        and "LANDING" in bridge_state_names
        and bridge_state_names.index("HOME_HOVER")
        < bridge_state_names.index("LANDING")
    )
    transition_actual = None
    if transition_odom_positions:
        transition_z = [
            float(position[2]) for position in transition_odom_positions
        ]
        transition_actual = {
            "start": [
                float(value) for value in transition_odom_positions[0]
            ],
            "end": [
                float(value) for value in transition_odom_positions[-1]
            ],
            "minimum_z": min(transition_z),
            "maximum_z": max(transition_z),
            "maximum_upward_sample_step": max(
                (
                    next_z - z
                    for z, next_z in zip(transition_z, transition_z[1:])
                ),
                default=0.0,
            ),
        }
    return_home_actual = None
    if return_home_odom_positions:
        return_z = [
            float(position[2]) for position in return_home_odom_positions
        ]
        return_home_actual = {
            "start": [
                float(value) for value in return_home_odom_positions[0]
            ],
            "end": [
                float(value) for value in return_home_odom_positions[-1]
            ],
            "minimum_z": min(return_z),
            "maximum_z": max(return_z),
            "maximum_climb_above_start": (
                max(return_z) - return_z[0]
            ),
        }

    missing_topics = [
        topic for topic in REQUIRED_TOPICS if topic_counts.get(topic, 0) == 0
    ]
    control_connections = {
        topic: sorted(connection_publishers.get(topic, set()))
        for topic in CONTROL_TOPICS
    }
    raw_publishers = control_connections["/mavros/setpoint_raw/local"]
    other_control_publishers = {
        topic: publishers
        for topic, publishers in control_connections.items()
        if topic != "/mavros/setpoint_raw/local" and publishers
    }

    entry_gate_sector_valid = bool(entry_gates) and all(
        re.fullmatch(
            rf"ENTRY_GATE_s{expected_entry_sector}(?:_c\d+)?",
            item["id"],
        )
        for item in entry_gates
    )
    result = {
        "bag": path,
        "start_time": round(start, 3),
        "end_time": round(end, 3),
        "duration": round(end - start, 3),
        "message_count": sum(topic_counts.values()),
        "missing_required_topics": missing_topics,
        "mission_states": mission_states,
        "bridge_states": bridge_states,
        "entry_gates": entry_gates,
        "expected_entry_sector": expected_entry_sector,
        "adaptive_entry_gate_sector_valid": entry_gate_sector_valid,
        "fixed_entry_gate_valid": entry_gate_sector_valid,
        "ascent_targets": ascent_targets,
        "vertical_ascent_valid": vertical_ascent_valid,
        "vertical_ascent_actual_max_xy_error": ascent_xy_error,
        "layer_transition_targets": layer_transition_targets,
        "layer_transition_valid": layer_transition_valid,
        "layer_transition_command_same_xy": transition_command_same_xy,
        "layer_transition_actual_max_xy_error": transition_xy_error,
        "layer_transition_actual": transition_actual,
        "exit_gate_targets": exit_gate_targets,
        "exit_gate_actual_sample_count": len(exit_gate_odom_positions),
        "dynamic_lower_exit_gate_valid": dynamic_lower_exit_gate_valid,
        "normal_return_targets": normal_return_targets,
        "direct_normal_return_selected": direct_normal_return_selected,
        "return_home_planning_goals": return_home_planning_goals,
        "return_home_actual": return_home_actual,
        "tower_facing_yaw_error": {
            "count": len(tower_yaw_errors),
            "maximum": max(tower_yaw_errors) if tower_yaw_errors else None,
            "p95": percentile(tower_yaw_errors, 95.0),
        },
        "sector_targets": sector_targets,
        "sector_visits": sector_visits,
        "unique_sector_ids": unique_sector_ids,
        "repeated_sector_ids": repeated_sector_ids,
        "closed_lap_count": closed_lap_count,
        "closed_layers": closed_layers,
        "closed_lap_valid": closed_lap_valid,
        "return_egress_selected": return_egress_selected,
        "return_egress_targets": return_egress_targets,
        "return_home_radial_valid": return_home_radial_valid,
        "home_hover_before_landing": home_hover_before_landing,
        "current_sector_events": current_sector_events,
        "cancel_count": cancel_count,
        "planner": {
            "states": dict(planner_states),
            "reasons": dict(planner_reasons),
            "max_consecutive_failures": planner_max_failures,
            "flag_counts": dict(planner_flag_counts),
        },
        "tracking_error": {
            "count": len(tracking_errors),
            "maximum": max(tracking_errors) if tracking_errors else None,
            "mean": statistics.fmean(tracking_errors)
            if tracking_errors
            else None,
            "p95": percentile(tracking_errors, 95.0),
            "limit": tracking_error_limit,
            "protection_duration": tracking_error_duration,
            "above_limit_count": tracking_above_limit_count,
            "maximum_continuous_above_limit_duration": (
                tracking_max_continuous_duration
            ),
        },
        "mavros_speed": {
            "count": len(speeds),
            "maximum": max(speeds) if speeds else None,
            "p95": percentile(speeds, 95.0),
        },
        "raw_type_masks": {str(key): value for key, value in raw_masks.items()},
        "control_publishers": control_connections,
        "raw_control_unique_to_bridge": raw_publishers
        == ["/ego_mavros_bridge"],
        "other_control_publishers": other_control_publishers,
        "frames": {
            topic: sorted(values) for topic, values in frames.items()
        },
        "sampled_occupancy_clearance": {
            "stride": occupancy_stride,
            "sample_count": occupancy_samples,
            "minimum": (
                occupancy_min_distance
                if math.isfinite(occupancy_min_distance)
                else None
            ),
            "time": (
                round(occupancy_min_time, 3)
                if occupancy_min_time is not None
                else None
            ),
        },
        "final_mavros_state": final_mavros,
        "final_extended_state": final_extended,
        "final_mavros_position": (
            [float(value) for value in latest_pose]
            if latest_pose is not None
            else None
        ),
    }
    result["pass_conditions"] = {
        "mission_done": bool(mission_states)
        and mission_states[-1]["state"] == "DONE",
        "bridge_done": bool(bridge_states)
        and bridge_states[-1]["state"] == "DONE",
        "disarmed": final_mavros is not None and not final_mavros["armed"],
        "on_ground": final_extended is not None
        and final_extended["landed_state"] == 1,
        "no_missing_required_topics": not missing_topics,
        "closed_lap_completed": closed_lap_valid,
        "entry_gate_stays_in_configured_sector": (
            result["adaptive_entry_gate_sector_valid"]
        ),
        "fixed_xy_vertical_ascent": vertical_ascent_valid,
        "actual_vertical_ascent_xy_stable": (
            ascent_xy_error is not None and ascent_xy_error < 0.5
        ),
        "ordered_direct_layer_transition": layer_transition_valid,
        "actual_layer_transition_xy_stable": (
            not transition_command_same_xy
            or (
                transition_xy_error is not None
                and transition_xy_error < 0.5
            )
        ),
        "tower_facing_yaw_stable": (
            bool(tower_yaw_errors)
            and percentile(tower_yaw_errors, 95.0) < 0.35
        ),
        "dynamic_lower_exit_gate": dynamic_lower_exit_gate_valid,
        "direct_normal_return_without_far_egress": (
            direct_normal_return_selected
        ),
        "return_home_does_not_command_layer_climb": (
            bool(return_home_planning_goals)
            and max(
                goal["position"][2]
                for goal in return_home_planning_goals
            )
            < 22.0
        ),
        "home_hover_precedes_auto_land": home_hover_before_landing,
        "no_planner_collision_or_emergency": not planner_flag_counts,
        "no_sustained_tracking_error": (
            tracking_max_continuous_duration < tracking_error_duration
        ),
        "raw_type_mask_zero": set(raw_masks) == {0},
        "unique_control_output": (
            raw_publishers == ["/ego_mavros_bridge"]
            and not other_control_publishers
        ),
    }
    result["passed"] = all(result["pass_conditions"].values())
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument(
        "--occupancy-stride",
        type=int,
        default=20,
        help="analyze every Nth inflated occupancy message",
    )
    parser.add_argument(
        "--tracking-error-limit",
        type=float,
        default=1.0,
        help="bridge tracking-error threshold in metres",
    )
    parser.add_argument(
        "--tracking-error-duration",
        type=float,
        default=1.0,
        help="bridge sustained-error protection duration in seconds",
    )
    parser.add_argument(
        "--entry-sector",
        type=int,
        default=7,
        choices=range(1, 9),
        help="expected user-facing ENTRY_GATE sector (default: 7)",
    )
    parser.add_argument("--output")
    args = parser.parse_args()
    if args.occupancy_stride < 1:
        parser.error("--occupancy-stride must be positive")
    if args.tracking_error_limit <= 0.0:
        parser.error("--tracking-error-limit must be positive")
    if args.tracking_error_duration <= 0.0:
        parser.error("--tracking-error-duration must be positive")
    result = audit(
        args.bag,
        args.occupancy_stride,
        args.tracking_error_limit,
        args.tracking_error_duration,
        args.entry_sector,
    )
    payload = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        output = require_runtime_artifact_path(Path(args.output))
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.write("\n")
    print(payload)


if __name__ == "__main__":
    main()
