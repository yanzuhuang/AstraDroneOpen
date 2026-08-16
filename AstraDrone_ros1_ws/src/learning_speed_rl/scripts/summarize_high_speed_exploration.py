#!/usr/bin/env python3
"""Summarize the UAV1 high-speed exploration and conditional boundary retry."""

import argparse
import csv
import json
import math
import os
import re

import rosbag


RUN_SPECS = (
    (0.50, "v050", "0.50"),
    (1.00, "v100", "1.00"),
    (2.00, "v200", "2.00 first run"),
    (2.00, "v200_retry", "2.00 retry"),
    (1.50, "v150", "1.50 conditional"),
)
TOLERANCE = 0.005
TRACKING_THRESHOLDS = (0.5, 0.8, 1.0)
TOWER_CENTER_XY = (-10.0551, 19.7104)


def nested(item, path, default=None):
    for key in path.split("."):
        if not isinstance(item, dict) or key not in item:
            return default
        item = item[key]
    return item


def finite_close(value, expected):
    return (
        isinstance(value, (int, float))
        and math.isfinite(value)
        and abs(float(value) - expected) <= TOLERANCE
    )


def threshold_key(threshold):
    return "over_{:.1f}m".format(threshold).replace(".", "_")


def rotate_body_to_world(vector, quaternion):
    """Rotate a body-frame vector by a normalized xyzw quaternion."""
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm <= 1.0e-12:
        return None
    x, y, z, w = (value / norm for value in (x, y, z, w))
    vx, vy, vz = vector
    # q * v * q^-1, expanded as a rotation matrix.
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y - z * w) * vy
        + 2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx
        + 2.0 * (y * z + x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


def motion_context(mission_state, odom, actual_velocity_body):
    if mission_state in ("SEGMENTED_CLIMB", "STAGING", "ENTRY_GATE_TRANSIT"):
        return "ENTRY"
    if mission_state == "GO_TO_EXIT_GATE":
        return "EXIT"
    if mission_state in ("NORMAL_RETURN", "RETURN_HOME", "FAILURE_LANDING"):
        return "RETURN_OR_LANDING"
    if mission_state in ("TARGET_LOCKED", "EVALUATING", "HOLDING", "RECOVERING"):
        return "ORBIT_TURN_OR_HOLD"
    if mission_state != "NAVIGATING" or odom is None:
        return "OTHER_OR_UNRESOLVED"

    world_velocity = rotate_body_to_world(actual_velocity_body, odom[1])
    if world_velocity is None:
        return "ORBIT_NAVIGATION_UNRESOLVED"
    dx = odom[0][0] - TOWER_CENTER_XY[0]
    dy = odom[0][1] - TOWER_CENTER_XY[1]
    radius = math.hypot(dx, dy)
    horizontal_speed = math.hypot(world_velocity[0], world_velocity[1])
    if radius <= 1.0e-6 or horizontal_speed <= 1.0e-6:
        return "ORBIT_HOLD_OR_LOW_SPEED"
    radial_speed = abs((world_velocity[0] * dx + world_velocity[1] * dy) / radius)
    return (
        "TANGENTIAL_ORBIT"
        if radial_speed / horizontal_speed <= 0.35
        else "ORBIT_TURN_OR_RADIAL_TRANSITION"
    )


def bag_evidence(path):
    evidence = {
        "mission_states": [],
        "final_armed": None,
        "final_px4_mode": None,
        "final_landed_state": None,
        "final_landed_state_name": "UNKNOWN",
        "first_terminal_failure_reason": None,
        "current_inflated_occupancy_samples": 0,
        "current_inflated_occupancy_samples_by_mission_state": {},
        "planner_status_samples": 0,
        "planner_failure_status_samples": 0,
        "planner_failure_episodes": 0,
        "planner_failure_status_samples_by_reason": {},
        "planner_failure_status_samples_by_mission_state": {},
        "tracking_threshold_crossings": {
            threshold_key(threshold): None
            for threshold in TRACKING_THRESHOLDS
        },
    }
    if not os.path.isfile(path):
        return evidence

    mission_state = ""
    mission_started = False
    mission_terminal = False
    mission_start_stamp = None
    latest_requested = None
    latest_filtered = None
    latest_applied = None
    latest_odom = None
    terminal_patterns = (
        re.compile(r"\[BRIDGE\] TRACK_EGO -> HOLD: (tracking error .*? exceeds limit .*?)$"),
        re.compile(r"\[SECTOR_INSPECTION_DIAG\].* HOLD requested .* condition=\"([^\"]+)\""),
        re.compile(r"\[SECTOR_INSPECTION_TASK\].* -> FAILURE_LANDING: (.*?); layer="),
        re.compile(r"\[SECTOR_INSPECTION_TASK\].* -> ERROR: (.*?); layer="),
        re.compile(r"\[SECTOR_INSPECTION_TASK\].* -> ERROR: (.*)$"),
    )
    collision_by_state = {}
    planner_failure_by_reason = {}
    planner_failure_by_state = {}
    planner_failure_active = False
    planner_failure_reason = None
    with rosbag.Bag(path) as bag:
        for topic, message, _stamp in bag.read_messages(topics=[
            "/uav1/tower_mission/state",
            "/uav1/mavros/state",
            "/uav1/mavros/extended_state",
            "/uav1/planner/status",
            "/uav1/Odometry",
            "/uav1/learning_speed/raw_v_max",
            "/uav1/learning_speed/v_max",
            "/uav1/learning_speed/applied_v_max",
            "/uav1/learning_speed/observation_c",
            "/rosout",
        ]):
            if topic == "/uav1/tower_mission/state":
                mission_state = message.data
                mission_started = mission_started or mission_state != "WAIT_INPUTS"
                if mission_started and mission_start_stamp is None:
                    mission_start_stamp = _stamp.to_sec()
                mission_terminal = mission_state in ("DONE", "ERROR")
                if not evidence["mission_states"] or evidence["mission_states"][-1] != mission_state:
                    evidence["mission_states"].append(mission_state)
            elif topic == "/uav1/mavros/state":
                evidence["final_armed"] = bool(message.armed)
                evidence["final_px4_mode"] = str(message.mode)
            elif topic == "/uav1/mavros/extended_state":
                evidence["final_landed_state"] = int(message.landed_state)
            elif topic == "/uav1/Odometry":
                position = message.pose.pose.position
                orientation = message.pose.pose.orientation
                latest_odom = (
                    (float(position.x), float(position.y), float(position.z)),
                    (
                        float(orientation.x), float(orientation.y),
                        float(orientation.z), float(orientation.w),
                    ),
                )
            elif topic == "/uav1/learning_speed/raw_v_max":
                latest_requested = float(message.data)
            elif topic == "/uav1/learning_speed/v_max":
                latest_filtered = float(message.data)
            elif topic == "/uav1/learning_speed/applied_v_max":
                latest_applied = float(message.data)
            elif topic == "/uav1/learning_speed/observation_c":
                if not mission_started or mission_terminal or not bool(message.valid):
                    continue
                tracking_error = float(message.tracking_error_norm)
                velocity_body = (
                    float(message.actual_velocity_body.x),
                    float(message.actual_velocity_body.y),
                    float(message.actual_velocity_body.z),
                )
                actual_speed = math.sqrt(sum(value * value for value in velocity_body))
                observation_stamp = message.header.stamp.to_sec()
                for threshold in TRACKING_THRESHOLDS:
                    key = threshold_key(threshold)
                    if (
                        evidence["tracking_threshold_crossings"][key] is None
                        and tracking_error > threshold
                    ):
                        evidence["tracking_threshold_crossings"][key] = {
                            "threshold_m": threshold,
                            "observation_stamp_sec": observation_stamp,
                            "mission_elapsed_sec": (
                                max(0.0, observation_stamp - mission_start_stamp)
                                if mission_start_stamp is not None else None
                            ),
                            "tracking_error_norm_m": tracking_error,
                            "tracking_error_body_m": {
                                "x": float(message.tracking_error_body.x),
                                "y": float(message.tracking_error_body.y),
                                "z": float(message.tracking_error_body.z),
                            },
                            "requested_v_max_mps": latest_requested,
                            "filtered_v_max_mps": latest_filtered,
                            "applied_v_max_mps": (
                                latest_applied
                                if latest_applied is not None
                                else float(message.previous_v_max)
                            ),
                            "actual_velocity_body_mps": {
                                "x": velocity_body[0],
                                "y": velocity_body[1],
                                "z": velocity_body[2],
                            },
                            "actual_speed_norm_mps": actual_speed,
                            "mission_state": mission_state,
                            "motion_context": motion_context(
                                mission_state, latest_odom, velocity_body
                            ),
                            "motion_context_method": (
                                "mission state plus inferred radial/horizontal "
                                "speed ratio for NAVIGATING; ratio <=0.35 is tangential"
                            ),
                        }
            elif topic == "/uav1/planner/status":
                if not mission_started or mission_terminal:
                    continue
                evidence["planner_status_samples"] += 1
                failed = (
                    not bool(message.last_plan_success)
                    and str(message.failure_reason) not in ("", "NONE")
                )
                if failed:
                    reason = str(message.failure_reason)
                    evidence["planner_failure_status_samples"] += 1
                    planner_failure_by_reason[reason] = (
                        planner_failure_by_reason.get(reason, 0) + 1
                    )
                    state = mission_state or "UNKNOWN"
                    planner_failure_by_state[state] = (
                        planner_failure_by_state.get(state, 0) + 1
                    )
                    if not planner_failure_active or reason != planner_failure_reason:
                        evidence["planner_failure_episodes"] += 1
                    planner_failure_active = True
                    planner_failure_reason = reason
                else:
                    planner_failure_active = False
                    planner_failure_reason = None
                if bool(message.current_position_in_collision):
                    evidence["current_inflated_occupancy_samples"] += 1
                    key = mission_state or "UNKNOWN"
                    collision_by_state[key] = collision_by_state.get(key, 0) + 1
            elif topic == "/rosout" and evidence["first_terminal_failure_reason"] is None:
                text = str(message.msg)
                for pattern in terminal_patterns:
                    match = pattern.search(text)
                    if match:
                        evidence["first_terminal_failure_reason"] = match.group(1).strip()
                        break

    landed_names = {
        0: "UNDEFINED",
        1: "ON_GROUND",
        2: "IN_AIR",
        3: "TAKEOFF",
        4: "LANDING",
    }
    evidence["final_landed_state_name"] = landed_names.get(
        evidence["final_landed_state"], "UNKNOWN"
    )
    evidence["current_inflated_occupancy_samples_by_mission_state"] = dict(
        sorted(collision_by_state.items())
    )
    evidence["planner_failure_status_samples_by_reason"] = dict(
        sorted(planner_failure_by_reason.items())
    )
    evidence["planner_failure_status_samples_by_mission_state"] = dict(
        sorted(planner_failure_by_state.items())
    )
    return evidence


def failure_category(reason):
    text = (reason or "").lower()
    if any(token in text for token in ("clearance", "collision", "occupancy", "emergency", "safety")):
        return "safety"
    if "tracking" in text:
        return "tracking"
    if any(token in text for token in ("planner", "trajectory", "feasible", "replan", "a star")):
        return "EGO planning"
    if "timeout" in text or "no progress" in text:
        return "mission timeout"
    return "other"


def summarize_run(root, speed, run_id, label):
    run_dir = os.path.join(root, run_id)
    summary_path = os.path.join(run_dir, "run_summary.json")
    if not os.path.isfile(summary_path):
        return {
            "run_id": run_id,
            "label": label,
            "v_max_mps": speed,
            "executed": False,
            "result": "NOT RUN",
            "failure_reason": None,
        }
    with open(summary_path, encoding="utf-8") as stream:
        source = json.load(stream)
    preflight_path = os.path.join(run_dir, "speed_chain_preflight.json")
    preflight = {}
    if os.path.isfile(preflight_path):
        with open(preflight_path, encoding="utf-8") as stream:
            preflight = json.load(stream)
    evidence = bag_evidence(os.path.join(run_dir, "high_speed.bag"))

    requested_mean = nested(source, "velocity.requested_source_mps.mean")
    filtered_mean = nested(source, "velocity.filtered_v_max_mps.mean")
    applied_mean = nested(source, "velocity.applied_v_max_mps.mean")
    chain_valid = all((
        bool(preflight.get("topic_chain_valid")),
        bool(preflight.get("parameter_chain_valid")),
        finite_close(requested_mean, speed),
        finite_close(filtered_mean, speed),
        finite_close(applied_mean, speed),
        finite_close(preflight.get("ego_current_limit_mps"), speed),
    ))
    mission_success = bool(nested(source, "mission.success", False))
    mission_failure = bool(nested(source, "mission.failure", False))
    mission_done = bool(nested(source, "mission.done", False))
    orbit_complete = bool(nested(source, "mission.orbit_complete", False))
    emergency_stop = bool(nested(source, "safety.emergency_stop", False)) or bool(
        nested(source, "planner.emergency_trajectory", False)
    )
    landed = evidence["final_landed_state_name"] == "ON_GROUND"
    disarmed = evidence["final_armed"] is False
    passed = all((
        chain_valid,
        mission_success,
        not mission_failure,
        mission_done,
        orbit_complete,
        not emergency_stop,
        landed,
        disarmed,
    ))

    failure_reason = evidence["first_terminal_failure_reason"]
    if not passed and not failure_reason:
        if not chain_valid:
            failure_reason = "requested/filtered/applied/EGO speed-chain validation failed"
        elif not mission_done:
            failure_reason = "mission did not reach a terminal done result"
        elif mission_failure:
            failure_reason = "mission terminal failure (no detailed reason captured)"
        elif not orbit_complete:
            failure_reason = "orbit did not complete"
        elif emergency_stop:
            failure_reason = "EGO emergency stop became active"
        elif not landed or not disarmed:
            failure_reason = "final landing/disarm state was not ON_GROUND and disarmed"
        else:
            failure_reason = "mission success criteria were not all satisfied"

    states = evidence["mission_states"]
    non_return_collision_states = {
        state: count
        for state, count in evidence["current_inflated_occupancy_samples_by_mission_state"].items()
        if state not in ("NORMAL_RETURN", "RETURN_HOME", "LANDING", "DONE", "ERROR")
    }
    return {
        "run_id": run_id,
        "label": label,
        "v_max_mps": speed,
        "executed": True,
        "result": "PASS" if passed else "FAIL",
        "speed_chain_valid": chain_valid,
        "speed_chain": {
            "fixed_requested_v_max_mps": speed,
            "requested_mean_mps": requested_mean,
            "filtered_mean_mps": filtered_mean,
            "applied_mean_mps": applied_mean,
            "ego_current_limit_preflight_mps": preflight.get("ego_current_limit_mps"),
            "manager_ceiling_mps": preflight.get("manager_ceiling_mps"),
            "optimizer_ceiling_mps": preflight.get("optimizer_ceiling_mps"),
            "bspline_ceiling_mps": preflight.get("bspline_ceiling_mps"),
            "dynamic_speed_limit_maximum_mps": preflight.get("dynamic_speed_limit_maximum_mps"),
            "speed_safety_filter_maximum_mps": preflight.get("speed_safety_filter_maximum_mps"),
            "fixed_speed_policy_mps": preflight.get("fixed_speed_policy_mps"),
        },
        "mission": {
            "success": mission_success,
            "failure": mission_failure,
            "done": mission_done,
            "orbit_complete": orbit_complete,
            "completion_time_sec": nested(source, "mission.completion_time_sec"),
            "entry_observed": "ENTRY_GATE_TRANSIT" in states,
            "exit_observed": "GO_TO_EXIT_GATE" in states,
            "normal_return_observed": "NORMAL_RETURN" in states,
            "return_home_observed": "RETURN_HOME" in states,
            "terminal_state": nested(source, "mission.terminal_state"),
        },
        "actual_velocity": {
            "body_x_mps": nested(source, "velocity.actual_velocity_body_x_mps"),
            "body_y_mps": nested(source, "velocity.actual_velocity_body_y_mps"),
            "body_z_mps": nested(source, "velocity.actual_velocity_body_z_mps"),
            "euclidean_speed_mps": nested(source, "velocity.actual_speed_observation_c_mps"),
            "semantic_note": "configured EGO v_max is checked per axis; Euclidean speed is measured separately",
        },
        "tracking_error_norm_m": nested(source, "tracking.tracking_error_norm_m"),
        "tracking_error_body_m": {
            "x": nested(source, "tracking.tracking_error_body_x_m"),
            "y": nested(source, "tracking.tracking_error_body_y_m"),
            "z": nested(source, "tracking.tracking_error_body_z_m"),
        },
        "tracking_threshold_crossings": evidence["tracking_threshold_crossings"],
        "clearance": {
            "minimum_raw_m": nested(source, "safety.minimum_raw_filtered_point_distance_m"),
            "minimum_inflated_center_m": nested(source, "safety.minimum_ego_inflated_occupied_center_distance_m"),
            "raw_representation": nested(source, "safety.raw_clearance_representation"),
            "inflated_representation": nested(source, "safety.inflated_clearance_representation"),
        },
        "planner": {
            "failure": bool(nested(source, "planner.planner_failure", False)),
            "failure_count": int(nested(source, "planner.planner_failure_events", 0) or 0),
            "failure_count_semantics": (
                "recorder count of distinct PlannerStatus failure updates; "
                "use bag_failure_episode_count for contiguous episodes"
            ),
            "bag_failure_status_sample_count": evidence["planner_failure_status_samples"],
            "bag_failure_episode_count": evidence["planner_failure_episodes"],
            "bag_failure_status_samples_by_reason": (
                evidence["planner_failure_status_samples_by_reason"]
            ),
            "bag_failure_status_samples_by_mission_state": (
                evidence["planner_failure_status_samples_by_mission_state"]
            ),
            "replanning_count": int(nested(source, "planner.ego_replanning_count", 0) or 0),
            "emergency_stop": emergency_stop,
            "non_return_collision_samples_by_state": non_return_collision_states,
        },
        "observation_c_valid_ratio": nested(source, "observation_c.valid_ratio"),
        "final_vehicle_state": {
            "armed": evidence["final_armed"],
            "px4_mode": evidence["final_px4_mode"],
            "landed_state": evidence["final_landed_state_name"],
        },
        "failure_reason": failure_reason,
        "failure_category": failure_category(failure_reason) if not passed else None,
    }


def csv_value(item, path):
    value = nested(item, path)
    return "" if value is None else value


def format_number(value, digits=3):
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        return "--"
    return ("{:.%df}" % digits).format(value)


def crossing_detail(event):
    if not isinstance(event, dict):
        return "not observed"
    body = event.get("tracking_error_body_m", {})
    return (
        "t={elapsed}s err={error}m body=({x},{y},{z})m "
        "req/filt/applied={requested}/{filtered}/{applied}m/s "
        "actual={actual}m/s state={state} context={context}"
    ).format(
        elapsed=format_number(event.get("mission_elapsed_sec"), 3),
        error=format_number(event.get("tracking_error_norm_m"), 3),
        x=format_number(body.get("x"), 3),
        y=format_number(body.get("y"), 3),
        z=format_number(body.get("z"), 3),
        requested=format_number(event.get("requested_v_max_mps"), 3),
        filtered=format_number(event.get("filtered_v_max_mps"), 3),
        applied=format_number(event.get("applied_v_max_mps"), 3),
        actual=format_number(event.get("actual_speed_norm_mps"), 3),
        state=event.get("mission_state") or "UNKNOWN",
        context=event.get("motion_context") or "UNKNOWN",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root")
    args = parser.parse_args()
    root = os.path.abspath(args.root)
    os.makedirs(root, exist_ok=True)
    runs = [
        summarize_run(root, speed, run_id, label)
        for speed, run_id, label in RUN_SPECS
    ]
    by_id = {run["run_id"]: run for run in runs}
    original_runs = [by_id[run_id] for run_id in ("v050", "v100", "v200")]
    retry = by_id["v200_retry"]
    conditional = by_id["v150"]

    for run in original_runs:
        if not run["executed"]:
            run["result"] = "HISTORICAL RESULT MISSING"
            run["failure_reason"] = "preserved historical result is missing"
    if not retry["executed"]:
        retry["result"] = "NOT RUN - PENDING"
    if not conditional["executed"]:
        if retry["executed"] and retry["result"] == "PASS":
            conditional["result"] = "NOT RUN - 2.0 RETRY PASS"
            conditional["failure_reason"] = (
                "conditional 1.5 m/s run correctly skipped after 2.0 retry PASS"
            )
        else:
            conditional["result"] = "NOT RUN - PENDING"

    original_early_stop_correct = (
        [run["result"] for run in original_runs] == ["PASS", "PASS", "FAIL"]
    )
    if retry["executed"] and retry["result"] == "PASS":
        conditional_flow_correct = not conditional["executed"]
        boundary_refinement_complete = conditional_flow_correct
    elif retry["executed"] and retry["result"] == "FAIL":
        conditional_flow_correct = conditional["executed"]
        boundary_refinement_complete = conditional["executed"]
    else:
        conditional_flow_correct = False
        boundary_refinement_complete = False

    passed_speeds = [run["v_max_mps"] for run in runs if run["result"] == "PASS"]
    highest_verified = max(passed_speeds) if passed_speeds else None
    first_failure_repeated = retry["executed"] and retry["result"] == "FAIL"
    tracking_failure_repeated = (
        first_failure_repeated and retry.get("failure_category") == "tracking"
    )
    if retry["executed"] and retry["result"] == "PASS":
        current_boundary = "2.0 m/s itself shows clear run-to-run variability"
        minimum_next_experiment = (
            "repeat 2.0 m/s twice under unchanged settings to quantify its success rate"
        )
    elif retry["executed"] and retry["result"] == "FAIL" and conditional["executed"]:
        if conditional["result"] == "PASS":
            current_boundary = "1.5-2.0 m/s"
            minimum_next_experiment = (
                "repeat 1.5 m/s once under unchanged settings to check reproducibility; "
                "do not test a new speed before that result"
            )
        else:
            current_boundary = "1.0-1.5 m/s"
            minimum_next_experiment = (
                "test 1.25 m/s once, then repeat only the resulting boundary candidate"
            )
    else:
        current_boundary = "PENDING"
        minimum_next_experiment = "finish the authorized conditional retry sequence"

    summary = {
        "schema_version": "high_speed_exploration.v2",
        "historical_formal_three_uav": {
            "uav1_max_vel_mps": 0.20,
            "uav2_max_vel_mps": 0.20,
            "uav3_max_vel_mps": 0.20,
            "max_acc_mps2": 0.50,
        },
        "exploration": {
            "ordered_run_ids": [run_id for _speed, run_id, _label in RUN_SPECS],
            "exploration_ceiling_mps": 2.0,
            "max_acc_mps2": 0.50,
            "planning_horizon_m": 7.5,
            "original_early_stop_correct": original_early_stop_correct,
            "highest_verified_speed_mps": highest_verified,
            "first_failed_speed_mps": 2.0,
            "sac_action_range_decision_supported": False,
        },
        "boundary_refinement": {
            "rule": "run v200_retry; run v150 only if v200_retry FAIL",
            "conditional_flow_correct": conditional_flow_correct,
            "complete": boundary_refinement_complete,
            "two_mps_first_failure_repeated": first_failure_repeated,
            "two_mps_tracking_failure_repeated": tracking_failure_repeated,
            "current_speed_boundary": current_boundary,
            "minimum_next_experiment": minimum_next_experiment,
        },
        "runs": runs,
    }
    with open(os.path.join(root, "high_speed_summary.json"), "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, sort_keys=True)
        stream.write("\n")

    columns = [
        "v_max_mps", "run_id", "label", "executed", "result", "completion_time_sec",
        "requested_mean_mps", "filtered_mean_mps", "applied_mean_mps",
        "actual_speed_mean_mps", "actual_speed_p95_mps", "actual_speed_max_mps",
        "tracking_median_m", "tracking_p95_m", "tracking_max_m",
        "minimum_raw_clearance_m", "minimum_inflated_clearance_m",
        "planner_failure_count", "planner_failure_episode_count",
        "emergency_stop", "replanning_count",
        "observation_c_valid_ratio", "final_armed", "final_landed_state",
        "first_over_0_5m_elapsed_sec", "first_over_0_8m_elapsed_sec",
        "first_over_1_0m_elapsed_sec", "first_over_1_0m_mission_state",
        "first_over_1_0m_motion_context",
        "failure_reason",
    ]
    with open(os.path.join(root, "high_speed_runs.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for run in runs:
            writer.writerow({
                "v_max_mps": run["v_max_mps"],
                "run_id": run["run_id"],
                "label": run["label"],
                "executed": run["executed"],
                "result": run["result"],
                "completion_time_sec": csv_value(run, "mission.completion_time_sec"),
                "requested_mean_mps": csv_value(run, "speed_chain.requested_mean_mps"),
                "filtered_mean_mps": csv_value(run, "speed_chain.filtered_mean_mps"),
                "applied_mean_mps": csv_value(run, "speed_chain.applied_mean_mps"),
                "actual_speed_mean_mps": csv_value(run, "actual_velocity.euclidean_speed_mps.mean"),
                "actual_speed_p95_mps": csv_value(run, "actual_velocity.euclidean_speed_mps.p95"),
                "actual_speed_max_mps": csv_value(run, "actual_velocity.euclidean_speed_mps.max"),
                "tracking_median_m": csv_value(run, "tracking_error_norm_m.median"),
                "tracking_p95_m": csv_value(run, "tracking_error_norm_m.p95"),
                "tracking_max_m": csv_value(run, "tracking_error_norm_m.max"),
                "minimum_raw_clearance_m": csv_value(run, "clearance.minimum_raw_m"),
                "minimum_inflated_clearance_m": csv_value(run, "clearance.minimum_inflated_center_m"),
                "planner_failure_count": csv_value(run, "planner.failure_count"),
                "planner_failure_episode_count": csv_value(
                    run, "planner.bag_failure_episode_count"
                ),
                "emergency_stop": csv_value(run, "planner.emergency_stop"),
                "replanning_count": csv_value(run, "planner.replanning_count"),
                "observation_c_valid_ratio": csv_value(run, "observation_c_valid_ratio"),
                "final_armed": csv_value(run, "final_vehicle_state.armed"),
                "final_landed_state": csv_value(run, "final_vehicle_state.landed_state"),
                "first_over_0_5m_elapsed_sec": csv_value(
                    run, "tracking_threshold_crossings.over_0_5m.mission_elapsed_sec"
                ),
                "first_over_0_8m_elapsed_sec": csv_value(
                    run, "tracking_threshold_crossings.over_0_8m.mission_elapsed_sec"
                ),
                "first_over_1_0m_elapsed_sec": csv_value(
                    run, "tracking_threshold_crossings.over_1_0m.mission_elapsed_sec"
                ),
                "first_over_1_0m_mission_state": csv_value(
                    run, "tracking_threshold_crossings.over_1_0m.mission_state"
                ),
                "first_over_1_0m_motion_context": csv_value(
                    run, "tracking_threshold_crossings.over_1_0m.motion_context"
                ),
                "failure_reason": run.get("failure_reason") or "",
            })

    lines = [
        "# UAV1 worksite 高速 EGO `v_max` 阶梯探索报告",
        "",
        "## 基线与范围",
        "",
        "原正式三机真实配置：UAV1/UAV2/UAV3 的 `max_vel` 均为 0.20 m/s，",
        "`max_acc=0.50 m/s²`。原 EGO `max_vel` 是旧系统静态配置；",
        "SpeedSafetyFilter 由 Learning Speed 接入新增；FixedSpeedPolicy 由 baseline",
        "测试新增。本次仅在隔离的 UAV1 高速探索入口把允许规划速度 ceiling 开到",
        "2.0 m/s，不改变三机默认值，也不代表最终 SAC action range。",
        "",
        "正式链保持 `FixedSpeedPolicy -> SpeedSafetyFilter -> dynamic v_max -> ",
        "EGO manager + optimizer -> applied_v_max -> B-spline -> PX4`。clearance、",
        "inflation、collision、emergency stop、最低高度、地图占据、ENTRY/EXIT、",
        "waypoint、Observation 与 mission failure 条件均未放宽。",
        "",
        "## 结果",
        "",
        "| run | executed | PASS/FAIL | completion | actual p95 | tracking p95 | min clearance (raw / inflated) | failure reason |",
        "| :-- | :------: | :-------: | ---------: | ---------: | -----------: | ------------------------------: | :------------- |",
    ]
    for run in runs:
        completion = format_number(nested(run, "mission.completion_time_sec"), 3)
        actual_p95 = format_number(nested(run, "actual_velocity.euclidean_speed_mps.p95"), 3)
        tracking_p95 = format_number(nested(run, "tracking_error_norm_m.p95"), 3)
        raw = format_number(nested(run, "clearance.minimum_raw_m"), 3)
        inflated = format_number(nested(run, "clearance.minimum_inflated_center_m"), 3)
        reason = (run.get("failure_reason") or "--").replace("|", "\\|")
        lines.append(
            "| {} | {} | {} | {} s | {} m/s | {} m | {} / {} m | {} |".format(
                run["label"], "yes" if run["executed"] else "no",
                run["result"], completion, actual_p95, tracking_p95, raw, inflated, reason
            )
        )

    lines.extend([
        "",
        "| run | requested / filtered / applied | actual mean / p95 / max | tracking median / p95 / max | planner failure updates / replans | emergency | Obs C valid | armed / landed |",
        "| :-- | -----------------------------: | -----------------------: | ---------------------------: | -------------------------: | :-------: | ----------: | :---- |",
    ])
    for run in runs:
        requested = format_number(nested(run, "speed_chain.requested_mean_mps"), 3)
        filtered = format_number(nested(run, "speed_chain.filtered_mean_mps"), 3)
        applied = format_number(nested(run, "speed_chain.applied_mean_mps"), 3)
        actual_mean = format_number(nested(run, "actual_velocity.euclidean_speed_mps.mean"), 3)
        actual_p95 = format_number(nested(run, "actual_velocity.euclidean_speed_mps.p95"), 3)
        actual_max = format_number(nested(run, "actual_velocity.euclidean_speed_mps.max"), 3)
        tracking_median = format_number(nested(run, "tracking_error_norm_m.median"), 3)
        tracking_p95 = format_number(nested(run, "tracking_error_norm_m.p95"), 3)
        tracking_max = format_number(nested(run, "tracking_error_norm_m.max"), 3)
        planner_failures = csv_value(run, "planner.failure_count")
        replans = csv_value(run, "planner.replanning_count")
        emergency = csv_value(run, "planner.emergency_stop")
        observation_valid = format_number(nested(run, "observation_c_valid_ratio"), 3)
        final = "{}/{}".format(
            csv_value(run, "final_vehicle_state.armed"),
            csv_value(run, "final_vehicle_state.landed_state"),
        )
        lines.append(
            "| {} | {} / {} / {} | {} / {} / {} m/s | {} / {} / {} m | {} / {} | {} | {} | {} |".format(
                run["label"], requested, filtered, applied,
                actual_mean, actual_p95, actual_max,
                tracking_median, tracking_p95, tracking_max,
                planner_failures, replans, emergency, observation_valid, final,
            )
        )

    lines.append("")
    failure_runs = [
        run for run in runs
        if run.get("executed") and nested(run, "planner.failure_count", 0)
    ]
    if failure_runs:
        lines.extend([
            "Planner failure 列是 recorder 记录的不同 `PlannerStatus` failure update 数，",
            "不是彼此独立的 terminal failure 次数。bag 另按连续原因折叠为 episode：",
            "",
        ])
        for run in failure_runs:
            lines.append(
                "- `{}`: recorder updates={}，bag episodes={}，reasons={}，states={}；"
                "最终 mission result={}。".format(
                    run["run_id"],
                    nested(run, "planner.failure_count", 0),
                    nested(run, "planner.bag_failure_episode_count", 0),
                    json.dumps(
                        nested(run, "planner.bag_failure_status_samples_by_reason", {}),
                        ensure_ascii=False,
                        sort_keys=True,
                    ),
                    json.dumps(
                        nested(run, "planner.bag_failure_status_samples_by_mission_state", {}),
                        ensure_ascii=False,
                        sort_keys=True,
                    ),
                    run["result"],
                )
            )
        lines.append("")

    lines.extend([
        "`configured v_max` 与实际速度不是同一个量。当前 EGO optimizer 主要按",
        "`vx/vy/vz` 逐轴检查；报告中的 actual speed 是由 Observation C 的",
        "`actual_velocity_body.{x,y,z}` 另行计算的欧氏范数，不能把 fixed v_max",
        "表述成三维速度范数的严格上限。actual velocity、tracking error body",
        "各轴及其范数的完整统计保存在",
        "`high_speed_summary.json`。",
        "",
        "两次 2.00 run 都在 ENTRY 阶段约 22 s 被 tracking safety gate 终止，",
        "因此两次 clearance 都只覆盖起飞/进场和安全返航，不能与完整绕塔的",
        "minimum clearance 作同范围比较。",
        "",
        "## Tracking threshold 首次越界",
        "",
        "时刻相对各 run 的 mission start。body xyz、速度链、actual speed 和",
        "mission/motion context 来自同一 Observation C 样本附近的 bag 证据。",
        "NAVIGATING 中的切向/转弯标签由 odom 姿态及径向速度占水平速度比例推断；",
        "其他阶段直接使用 mission state。",
        "",
    ])
    for run in runs:
        if not run["executed"]:
            continue
        lines.append("- `{}`".format(run["run_id"]))
        for threshold in TRACKING_THRESHOLDS:
            key = threshold_key(threshold)
            lines.append(
                "  - first > {:.1f} m: {}".format(
                    threshold,
                    crossing_detail(nested(run, "tracking_threshold_crossings." + key)),
                )
            )

    lines.extend([
        "",
        "## 加速度、制动距离与 horizon",
        "",
        "保持 `a_max=0.50 m/s²` 时，按 `t_stop=v/a`、`d_stop=v²/(2a)` 的理想",
        "一维估算：0.50 m/s 为 1.0 s / 0.25 m，1.00 m/s 为 2.0 s / 1.0 m，",
        "1.50 m/s 为 3.0 s / 2.25 m，2.00 m/s 为 4.0 s / 4.0 m。相对 7.5 m",
        "planning horizon，制动距离分别占约 3.3%、13.3%、30.0%、53.3%。",
        "局部地图、曲线路径、逐轴约束、感知/重规划延迟和跟踪动态会进一步消耗余量。",
        "",
        "## 明确回答",
        "",
        "1. 2.0 m/s 第一次失败是否可重复：{}。".format(
            "是" if first_failure_repeated else (
                "否；retry PASS，显示明显运行间随机性"
                if retry["executed"] else "待运行"
            )
        ),
        "2. 第二次 2.0 速度链 requested/filtered/applied = {}/{}/{} m/s。".format(
            format_number(nested(retry, "speed_chain.requested_mean_mps"), 3),
            format_number(nested(retry, "speed_chain.filtered_mean_mps"), 3),
            format_number(nested(retry, "speed_chain.applied_mean_mps"), 3),
        ),
        "3. 第二次 2.0 actual mean/median/p95/max = {}/{}/{}/{} m/s；tracking mean/median/p95/max = {}/{}/{}/{} m。".format(
            format_number(nested(retry, "actual_velocity.euclidean_speed_mps.mean"), 3),
            format_number(nested(retry, "actual_velocity.euclidean_speed_mps.median"), 3),
            format_number(nested(retry, "actual_velocity.euclidean_speed_mps.p95"), 3),
            format_number(nested(retry, "actual_velocity.euclidean_speed_mps.max"), 3),
            format_number(nested(retry, "tracking_error_norm_m.mean"), 3),
            format_number(nested(retry, "tracking_error_norm_m.median"), 3),
            format_number(nested(retry, "tracking_error_norm_m.p95"), 3),
            format_number(nested(retry, "tracking_error_norm_m.max"), 3),
        ),
        "4. 若 retry FAIL，根因是否仍为 tracking：{}。".format(
            "是" if tracking_failure_repeated else (
                "否" if first_failure_repeated else "不适用"
            )
        ),
        "5. 1.5 m/s：{}。".format(conditional["result"]),
        "6. 当前最高验证通过速度：{}。".format(
            "NONE" if highest_verified is None else "{:.2f} m/s".format(highest_verified)
        ),
        "7. 当前 PASS/FAIL 边界：{}。".format(current_boundary),
        "8. 当前仍不足以确定 SAC 最大 action speed；单次 PASS 或两次 FAIL 都不能建立所需成功率。",
        "9. 最小必要下一步实验：{}；本轮不自动执行。".format(minimum_next_experiment),
        "",
        "```text",
        "2.0 M/S RETEST: {}".format(retry["result"]),
        "1.5 M/S TEST: {}".format(conditional["result"]),
        "",
        "HIGHEST VERIFIED SPEED: {}".format(
            "NONE" if highest_verified is None else "{:.2f} m/s".format(highest_verified)
        ),
        "CURRENT SPEED BOUNDARY: {}".format(current_boundary),
        "```",
        "",
    ])
    with open(os.path.join(root, "high_speed_exploration_report.md"), "w", encoding="utf-8") as stream:
        stream.write("\n".join(lines))

    print("wrote high-speed exploration summary under {}".format(root))


if __name__ == "__main__":
    main()
