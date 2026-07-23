#!/usr/bin/env python3
"""Audit one Stage 3 ROS1 flight bag and emit machine-readable JSON."""

import argparse
import json
import math
import re
import statistics
from collections import Counter, defaultdict

import numpy as np
import rosbag


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
                mission_states.append(transition_event(timestamp, message.data))
            elif topic == "/tower_mission/current_sector":
                current_sector_index = int(message.data)
                current_sector_events.append(
                    {"time": round(timestamp, 3), "index": current_sector_index}
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
                "/Odometry",
                "/stage3/cloud_registered_filtered",
                "/planning/goal",
                "/planning/pos_cmd",
            ):
                frames[topic].add(message.header.frame_id)
            elif topic == "/rosout":
                text = message.msg
                gate = re.search(
                    r"ENTRY_GATE selected: (ENTRY_GATE_a\d+)", text
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
                locked = re.search(
                    r"safe candidate locked: (s\d+_c\d+)", text
                )
                if initial:
                    sector_targets.append(
                        {
                            "time": round(timestamp, 3),
                            "sector_id": int(initial.group(1)),
                            "target_id": initial.group(2),
                            "visit_index": current_sector_index,
                        }
                    )
                elif locked:
                    target_id = locked.group(1)
                    sector_targets.append(
                        {
                            "time": round(timestamp, 3),
                            "sector_id": int(target_id.split("_")[0][1:]),
                            "target_id": target_id,
                            "visit_index": current_sector_index,
                        }
                    )

    sector_visits_by_index = {}
    for target in sector_targets:
        sector_visits_by_index.setdefault(target["visit_index"], target)
    sector_visits = [
        sector_visits_by_index[index]
        for index in sorted(sector_visits_by_index)
    ]
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
        "sector_targets": sector_targets,
        "sector_visits": sector_visits,
        "unique_sector_ids": unique_sector_ids,
        "repeated_sector_ids": repeated_sector_ids,
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
        "no_duplicate_sector_target_ids": not repeated_sector_ids,
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
    )
    payload = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.write("\n")
    print(payload)


if __name__ == "__main__":
    main()
