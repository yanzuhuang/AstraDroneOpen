#!/usr/bin/env python3
"""Summarize progressive high-speed qualification without defining reward."""

import argparse
import bisect
import csv
import json
import math
from collections import Counter
from pathlib import Path
import re

import numpy as np
import rosbag
from pyulog import ULog
import yaml


SPEEDS = (1.75, 2.0, 2.5, 3.0, 3.5)
EXPECTED_INVALID_PREFIXES = (
    "trajectory_unavailable",
    "lidar_surrogate_invalid:insufficient_history:",
    "lidar_surrogate_invalid:timestamp_sync:cloud_newer_than_pose_history",
)
ACTIVE_STATES = {
    "ENTRY_GATE_TRANSIT", "GO_TO_EXIT_GATE", "LAYER_TRANSITION",
    "NAVIGATING", "NORMAL_RETURN", "RECOVERING", "RELOCATING",
    "RETURN_EGRESS", "STAGING_POINT",
}


def number(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def percentile(values, fraction):
    values = sorted(value for value in values if value is not None)
    if not values:
        return None
    position = (len(values) - 1) * fraction
    low, high = int(math.floor(position)), int(math.ceil(position))
    if low == high:
        return values[low]
    return values[low] + (values[high] - values[low]) * (position - low)


def stats(values):
    values = [value for value in values if value is not None]
    return {
        "count": len(values),
        "min": min(values) if values else None,
        "mean": sum(values) / len(values) if values else None,
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def norm3(vector):
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def in_intervals(stamp, intervals):
    return any(start <= stamp < end for start, end in intervals)


def state_intervals(events, wanted):
    intervals = []
    for index, (start, state) in enumerate(events):
        end = events[index + 1][0] if index + 1 < len(events) else math.inf
        if state.strip('"') == wanted:
            intervals.append((start, end))
    return intervals


def de_boor(control, knots, order, value):
    n = len(control) - 1
    m = n + order + 1
    bounded = min(max(knots[order], value), knots[m - order])
    index = order
    while index + 1 < len(knots) and knots[index + 1] < bounded:
        index += 1
    work = [control[index - order + item].copy()
            for item in range(order + 1)]
    for level in range(1, order + 1):
        for item in range(order, level - 1, -1):
            denominator = (knots[item + 1 + index - level]
                           - knots[item + index - order])
            alpha = ((bounded - knots[item + index - order]) / denominator
                     if denominator else 0.0)
            work[item] = (1.0 - alpha) * work[item - 1] + alpha * work[item]
    return work[order]


def infer_random_fallbacks(log_text):
    """Infer fallback calls from the fixed three-attempt FSM call order."""
    inferred = 0
    failures = 0
    for line in log_text.splitlines():
        if "refine_success=0" in line:
            failures += 1
        elif "refine_success=1" in line:
            if failures >= 2:
                inferred += 1
            failures = 0
        elif "from REPLAN_TRAJ to REPLAN_TRAJ" in line:
            if failures >= 3:
                inferred += 1
            failures = 0
    return inferred


def analyze_csv(run_dir, summary):
    rows = list(csv.DictReader((run_dir / "calibration_samples.csv").open(
        newline="", encoding="utf-8")))
    valid_rows = [row for row in rows if row.get("observation_valid") == "1"]
    active_rows = [row for row in rows if row.get("mission_state") in ACTIVE_STATES]
    active_valid = [row for row in active_rows if row.get("observation_valid") == "1"]

    def values(field, source=valid_rows):
        return [number(row.get(field)) for row in source]

    invalid_reasons = summary.get("observation_c", {}).get("invalid_reasons", {})
    unexplained = {
        reason: count for reason, count in invalid_reasons.items()
        if not reason.startswith(EXPECTED_INVALID_PREFIXES)
    }
    lower_reasons = {reason.lower(): count for reason, count in invalid_reasons.items()}
    timestamp_mismatch = sum(
        count for reason, count in lower_reasons.items()
        if "trajectory timestamp mismatch" in reason
        or "fusion_invalid:trajectory" in reason)
    kinematic_gap = sum(
        count for reason, count in lower_reasons.items()
        if "kinematic_interpolation_gap_too_large" in reason
        or "kinematic gap" in reason)

    return {
        "rows": len(rows),
        "valid_rows": len(valid_rows),
        "active_rows": len(active_rows),
        "active_valid_rows": len(active_valid),
        "requested_v_max": stats(values("latest_requested_v_max_mps", rows)),
        "filtered_v_max": stats(values("latest_filtered_v_max_mps", rows)),
        "applied_v_max": stats(values("latest_applied_v_max_mps", rows)),
        "actual_speed_mps": stats(values("actual_speed_mps")),
        "tracking_error_m": stats(values("tracking_error_norm_m")),
        "nearest_obstacle_m": stats(values("nearest_obstacle_distance_m")),
        "density_proxy": stats(values("known_obstacle_bin_fraction")),
        "clutter_proxy_bins": stats(values("known_obstacle_bin_count")),
        "raw_observation_valid_ratio": (
            len(valid_rows) / len(rows) if rows else None),
        "training_active_valid_ratio": (
            len(active_valid) / len(active_rows) if active_rows else None),
        "invalid_reasons": invalid_reasons,
        "unexplained_invalid": unexplained,
        "trajectory_timestamp_mismatch": timestamp_mismatch,
        "kinematic_gap": kinematic_gap,
    }


def analyze_bag(run_dir, bridge_velocity_limit, bridge_acceleration_limit):
    bag_path = run_dir / "control_chain.bag"
    topics = {
        "mission_state": "/uav1/tower_mission/state",
        "mission_done": "/uav1/tower_mission/mission_done",
        "bridge_state": "/uav1/ego_mavros_bridge/state",
        "position_command": "/uav1/planning/pos_cmd",
        "raw_setpoint": "/uav1/mavros/setpoint_raw/local",
        "tracking": "/uav1/ego_mavros_bridge/tracking_error",
        "bspline": "/uav1/planning/bspline",
        "planner": "/uav1/planner/status",
        "mavros_state": "/uav1/mavros/state",
        "extended": "/uav1/mavros/extended_state",
    }
    reverse = {topic: name for name, topic in topics.items()}
    data = {name: [] for name in topics}
    with rosbag.Bag(str(bag_path)) as bag:
        bag_start, bag_end = bag.get_start_time(), bag.get_end_time()
        for topic, message, stamp in bag.read_messages(topics=list(reverse)):
            data[reverse[topic]].append((stamp.to_sec(), message))

    bridge_events = [(stamp, message.data) for stamp, message in data["bridge_state"]]
    track_intervals = state_intervals(bridge_events, "TRACK_EGO")
    commands = [(stamp, message) for stamp, message in data["position_command"]
                if in_intervals(stamp, track_intervals)]
    raws = [(stamp, message) for stamp, message in data["raw_setpoint"]
            if in_intervals(stamp, track_intervals)]
    tracking = [float(message.data) for stamp, message in data["tracking"]
                if in_intervals(stamp, track_intervals)]

    command_speed = [norm3((message.velocity.x, message.velocity.y,
                            message.velocity.z)) for _, message in commands]
    command_acceleration = [norm3((message.acceleration.x,
                                   message.acceleration.y,
                                   message.acceleration.z))
                            for _, message in commands]
    raw_speed = [norm3((message.velocity.x, message.velocity.y,
                        message.velocity.z)) for _, message in raws]
    raw_acceleration = [norm3((message.acceleration_or_force.x,
                               message.acceleration_or_force.y,
                               message.acceleration_or_force.z))
                        for _, message in raws]
    command_stamps = [stamp for stamp, _ in commands]
    velocity_difference, acceleration_difference = [], []
    velocity_saturation_count, acceleration_saturation_count = 0, 0
    for stamp, raw in raws:
        index = bisect.bisect_right(command_stamps, stamp) - 1
        if index < 0:
            continue
        command = commands[index][1]
        velocity_difference.append(norm3((
            command.velocity.x - raw.velocity.x,
            command.velocity.y - raw.velocity.y,
            command.velocity.z - raw.velocity.z)))
        acceleration_difference.append(norm3((
            command.acceleration.x - raw.acceleration_or_force.x,
            command.acceleration.y - raw.acceleration_or_force.y,
            command.acceleration.z - raw.acceleration_or_force.z)))
        command_velocity_norm = norm3((command.velocity.x, command.velocity.y,
                                       command.velocity.z))
        raw_velocity_norm = norm3((raw.velocity.x, raw.velocity.y,
                                   raw.velocity.z))
        command_acceleration_norm = norm3((
            command.acceleration.x, command.acceleration.y,
            command.acceleration.z))
        raw_acceleration_norm = norm3((
            raw.acceleration_or_force.x, raw.acceleration_or_force.y,
            raw.acceleration_or_force.z))
        if (command_velocity_norm > bridge_velocity_limit + 0.005
                and abs(raw_velocity_norm - bridge_velocity_limit) <= 0.005):
            velocity_saturation_count += 1
        if (command_acceleration_norm > bridge_acceleration_limit + 0.005
                and abs(raw_acceleration_norm - bridge_acceleration_limit) <= 0.005):
            acceleration_saturation_count += 1

    bspline_rows = []
    for stamp, message in data["bspline"]:
        control = np.asarray([[point.x, point.y, point.z]
                              for point in message.pos_pts], dtype=float)
        knots = np.asarray(message.knots, dtype=float)
        order = int(message.order)
        start, end = knots[order], knots[len(control)]
        sample_times = np.linspace(start, end, 201)
        sampled = np.asarray([
            de_boor(control, knots, order, value) for value in sample_times])
        bspline_rows.append({
            "bag_stamp_sec": stamp,
            "trajectory_id": int(message.traj_id),
            "start_stamp_sec": message.start_time.to_sec(),
            "control_point_min_z_m": float(np.min(control[:, 2])),
            "control_point_max_z_m": float(np.max(control[:, 2])),
            "sampled_min_z_m": float(np.min(sampled[:, 2])),
            "sampled_max_z_m": float(np.max(sampled[:, 2])),
        })
    bspline_path = run_dir / "bspline_metrics.csv"
    with bspline_path.open("w", newline="", encoding="utf-8") as stream:
        fields = list(bspline_rows[0]) if bspline_rows else [
            "bag_stamp_sec", "trajectory_id", "start_stamp_sec",
            "control_point_min_z_m", "control_point_max_z_m",
            "sampled_min_z_m", "sampled_max_z_m"]
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(bspline_rows)

    mission_events = [(stamp, message.data.strip('"'))
                      for stamp, message in data["mission_state"]]
    mission_start = next((stamp for stamp, state in mission_events
                          if state not in ("WAIT_INPUTS", "DONE", "ERROR")), None)
    mission_end = next((stamp for stamp, message in data["mission_done"]
                        if bool(message.data)), None)
    current_samples = [bool(message.current_position_in_collision)
                       for _, message in data["planner"]]
    current_episodes = sum(
        value and (index == 0 or not current_samples[index - 1])
        for index, value in enumerate(current_samples))
    final_state = data["mavros_state"][-1][1] if data["mavros_state"] else None
    final_extended = data["extended"][-1][1] if data["extended"] else None
    log_text = (run_dir / "roslaunch.log").read_text(
        encoding="utf-8", errors="replace")
    return {
        "bag_path": str(bag_path),
        "bag_size_bytes": bag_path.stat().st_size,
        "bag_duration_sec": bag_end - bag_start,
        "track_ego_duration_sec": sum(end - start for start, end in track_intervals
                                      if math.isfinite(end)),
        "position_command_speed_mps": stats(command_speed),
        "raw_setpoint_speed_mps": stats(raw_speed),
        "position_command_acceleration_mps2": stats(command_acceleration),
        "raw_setpoint_acceleration_mps2": stats(raw_acceleration),
        "position_command_to_raw_velocity_difference_mps": stats(
            velocity_difference),
        "position_command_to_raw_acceleration_difference_mps2": stats(
            acceleration_difference),
        "bridge_velocity_saturation_count": velocity_saturation_count,
        "bridge_velocity_saturation_ratio": (
            velocity_saturation_count / len(velocity_difference)
            if velocity_difference else None),
        "bridge_acceleration_saturation_count": acceleration_saturation_count,
        "bridge_acceleration_saturation_ratio": (
            acceleration_saturation_count / len(acceleration_difference)
            if acceleration_difference else None),
        "tracking_error_m": stats(tracking),
        "bspline_count": len(bspline_rows),
        "bspline_control_min_z_m": min(
            (row["control_point_min_z_m"] for row in bspline_rows), default=None),
        "bspline_control_max_z_m": max(
            (row["control_point_max_z_m"] for row in bspline_rows), default=None),
        "bspline_sampled_min_z_m": min(
            (row["sampled_min_z_m"] for row in bspline_rows), default=None),
        "bspline_sampled_max_z_m": max(
            (row["sampled_max_z_m"] for row in bspline_rows), default=None),
        "replan_calls": len(re.findall(r"\[drone 0 replan [0-9]+\]", log_text)),
        "random_fallbacks_inferred": infer_random_fallbacks(log_text),
        "current_position_in_occupancy_samples": sum(current_samples),
        "current_position_in_occupancy_episodes": current_episodes,
        "mission_execution_time_sec": (
            mission_end - mission_start
            if mission_start is not None and mission_end is not None else None),
        "mission_start_sec": mission_start,
        "mission_end_sec": mission_end,
        "final_vehicle": {
            "armed": bool(final_state.armed) if final_state else None,
            "mode": final_state.mode if final_state else None,
            "landed_state": int(final_extended.landed_state)
            if final_extended else None,
            "landed_state_semantics": "1=ON_GROUND",
        },
    }


def analyze_ulog(run_dir, start_sec, end_sec):
    paths = sorted((run_dir / "px4_ulog").glob("*.ulg"))
    if not paths:
        return {"available": False}
    path = paths[-1]
    ulog = ULog(str(path), message_name_filter_list=[
        "vehicle_local_position", "trajectory_setpoint"])
    datasets = {item.name: item.data for item in ulog.data_list}
    local = datasets.get("vehicle_local_position")
    trajectory = datasets.get("trajectory_setpoint")
    result = {
        "available": True,
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "dropout_count": len(ulog.dropouts),
        "parameters": {key: float(ulog.initial_parameters[key]) for key in (
            "MPC_XY_VEL_MAX", "MPC_Z_VEL_MAX_UP", "MPC_Z_VEL_MAX_DN",
            "MPC_ACC_HOR_MAX", "MPC_ACC_UP_MAX", "MPC_ACC_DOWN_MAX")},
    }
    if local is not None:
        times = local["timestamp"] / 1e6
        mask = np.ones(len(times), dtype=bool)
        if start_sec is not None and end_sec is not None:
            mask = (times >= start_sec) & (times <= end_sec)
        speed = np.linalg.norm(np.column_stack([
            local["vx"], local["vy"], local["vz"]]), axis=1)
        acceleration = np.linalg.norm(np.column_stack([
            local["ax"], local["ay"], local["az"]]), axis=1)
        result["actual_speed_mps"] = stats(speed[mask].tolist())
        result["actual_acceleration_mps2"] = stats(acceleration[mask].tolist())
    if trajectory is not None:
        times = trajectory["timestamp"] / 1e6
        mask = np.ones(len(times), dtype=bool)
        if start_sec is not None and end_sec is not None:
            mask = (times >= start_sec) & (times <= end_sec)
        command_acc = np.linalg.norm(np.column_stack([
            trajectory["acceleration[0]"], trajectory["acceleration[1]"],
            trajectory["acceleration[2]"]]), axis=1)
        result["px4_received_command_acceleration_mps2"] = stats(
            command_acc[mask].tolist())
    return result


def analyze_run(run_dir):
    run_dir = Path(run_dir).resolve()
    manifest = json.loads((run_dir / "experiment_manifest.json").read_text())
    summary = json.loads((run_dir / "run_summary.json").read_text())
    csv_metrics = analyze_csv(run_dir, summary)
    bridge_config = Path(manifest["bridge_config"])
    bridge = yaml.safe_load(bridge_config.read_text(encoding="utf-8"))
    bag_metrics = analyze_bag(
        run_dir, float(bridge["max_velocity"]),
        float(bridge["max_acceleration"]))
    # Bag and ULog share simulation time in this SITL chain.
    ulog_metrics = analyze_ulog(
        run_dir, bag_metrics["mission_start_sec"], bag_metrics["mission_end_sec"])
    mission = summary.get("mission", {})
    safety = summary.get("safety", {})
    passed = bool(mission.get("done") and mission.get("success")
                  and not safety.get("dangerous_terminal")
                  and bag_metrics["final_vehicle"]["armed"] is False
                  and bag_metrics["final_vehicle"]["landed_state"] == 1)
    result = {
        "schema_version": "high_speed_progressive_run_analysis_v1.0",
        "run_id": manifest["logical_run_id"],
        "run_dir": str(run_dir),
        "environment": manifest["environment"],
        "fixed_v_max_mps": float(manifest["fixed_v_max_mps"]),
        "speed_ceiling_mps": float(manifest["speed_ceiling_mps"]),
        "max_acc_mps2": float(manifest["max_acc_mps2"]),
        "planning_horizon_m": float(manifest["planning_horizon_m"]),
        "route_fingerprint": manifest["route_fingerprint"],
        "qualification_only": bool(manifest.get("qualification_only")),
        "pass": passed,
        "mission": mission,
        "safety": safety,
        "planner": summary.get("planner", {}),
        "transitions": summary.get("transitions", {}),
        "observation_c": summary.get("observation_c", {}),
        "csv_metrics": csv_metrics,
        "control_chain": bag_metrics,
        "px4_ulog": ulog_metrics,
        "reward_defined": summary.get("reward_defined"),
        "training_started": summary.get("training_started"),
    }
    output = run_dir / "qualification_run_analysis.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    flat = {
        "run_id": result["run_id"], "fixed_v_max_mps": result["fixed_v_max_mps"],
        "pass": result["pass"],
        "actual_speed_mean_mps": csv_metrics["actual_speed_mps"]["mean"],
        "actual_speed_p95_mps": csv_metrics["actual_speed_mps"]["p95"],
        "actual_speed_max_mps": csv_metrics["actual_speed_mps"]["max"],
        "tracking_mean_m": csv_metrics["tracking_error_m"]["mean"],
        "tracking_p95_m": csv_metrics["tracking_error_m"]["p95"],
        "tracking_max_m": csv_metrics["tracking_error_m"]["max"],
        "raw_valid_ratio": csv_metrics["raw_observation_valid_ratio"],
        "training_active_valid_ratio": csv_metrics["training_active_valid_ratio"],
        "nearest_obstacle_min_m": csv_metrics["nearest_obstacle_m"]["min"],
        "nearest_obstacle_mean_m": csv_metrics["nearest_obstacle_m"]["mean"],
        "nearest_obstacle_p95_m": csv_metrics["nearest_obstacle_m"]["p95"],
        "density_mean": csv_metrics["density_proxy"]["mean"],
        "density_p95": csv_metrics["density_proxy"]["p95"],
        "clutter_mean_bins": csv_metrics["clutter_proxy_bins"]["mean"],
        "clutter_p95_bins": csv_metrics["clutter_proxy_bins"]["p95"],
        "planner_failure_episodes": result["planner"].get("failure_episodes"),
        "replan_calls": bag_metrics["replan_calls"],
        "random_fallbacks_inferred": bag_metrics["random_fallbacks_inferred"],
        "current_position_in_occupancy_episodes": bag_metrics[
            "current_position_in_occupancy_episodes"],
        "mission_execution_time_sec": bag_metrics["mission_execution_time_sec"],
    }
    with (run_dir / "qualification_metrics.csv").open(
            "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat))
        writer.writeheader()
        writer.writerow(flat)
    return result


def fmt(value, digits=3):
    if value is None:
        return "n/a"
    return ("{:.%df}" % digits).format(value)


def aggregate(root, baseline_path, report_path):
    root = Path(root).resolve()
    analyses = []
    for speed in SPEEDS:
        code = round(speed * 100)
        matches = sorted(root.glob("hsq_B_v{:03d}_r01/qualification_run_analysis.json".format(code)))
        if matches:
            analyses.append(json.loads(matches[-1].read_text()))
        else:
            analyses.append({"fixed_v_max_mps": speed, "not_run": True})
    baseline = json.loads(Path(baseline_path).read_text())
    baseline_b = [run for run in baseline["runs"] if run.get("environment") == "B"]
    completed = [run for run in analyses if not run.get("not_run")]
    failure = next((run for run in completed if not run.get("pass")), None)
    stable = max((run["fixed_v_max_mps"] for run in completed if run.get("pass")), default=None)
    result = {
        "schema_version": "high_speed_progressive_qualification_v1.0",
        "environment": "B",
        "runs": analyses,
        "first_failure_speed_mps": failure.get("fixed_v_max_mps") if failure else None,
        "highest_stable_qualification_speed_mps": stable,
        "stopped_after_failure": bool(failure),
        "baseline_postfix_B": baseline_b,
        "reward_defined": False,
        "training_started": False,
    }
    (root / "progressive_qualification_analysis.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")

    rows = []
    for run in analyses:
        if run.get("not_run"):
            rows.append({"speed": run["fixed_v_max_mps"], "result": "NOT_RUN"})
            continue
        csvm, chain = run["csv_metrics"], run["control_chain"]
        rows.append({
            "speed": run["fixed_v_max_mps"],
            "result": "PASS" if run["pass"] else "FAIL",
            "actual": csvm["actual_speed_mps"],
            "tracking": csvm["tracking_error_m"],
            "raw_valid": csvm["raw_observation_valid_ratio"],
            "active_valid": csvm["training_active_valid_ratio"],
            "nearest": csvm["nearest_obstacle_m"],
            "density": csvm["density_proxy"],
            "clutter": csvm["clutter_proxy_bins"],
            "planner": run["planner"].get("failure_episodes"),
            "completion": chain["mission_execution_time_sec"],
        })
    lines = [
        "# 高速逐级 qualification 报告", "",
        "> Environment B / worksite dense route；每档一次，真实 failure 即停。  ",
        "> Reward 仍为 null，未定义或修改任何 Reward。", "",
        "## 1. 逐档结果", "",
        "| configured v_max | result | actual mean/p95/max | tracking mean/p95/max | raw valid | training-active valid | nearest min/mean/p95 | density mean/p95 | clutter mean/p95 | planner episodes | completion s |",
        "|---:|---|---|---|---:|---:|---|---|---|---:|---:|",
    ]
    for row in rows:
        if row["result"] == "NOT_RUN":
            lines.append("| {} | NOT_RUN | - | - | - | - | - | - | - | - | - |".format(row["speed"]))
            continue
        lines.append(
            "| {speed:.2f} | {result} | {am}/{ap}/{ax} | {tm}/{tp}/{tx} | {rv} | {av} | {nmin}/{nm}/{np} | {dm}/{dp} | {cm}/{cp} | {planner} | {completion} |".format(
                speed=row["speed"], result=row["result"],
                am=fmt(row["actual"]["mean"]), ap=fmt(row["actual"]["p95"]), ax=fmt(row["actual"]["max"]),
                tm=fmt(row["tracking"]["mean"]), tp=fmt(row["tracking"]["p95"]), tx=fmt(row["tracking"]["max"]),
                rv=fmt(row["raw_valid"]), av=fmt(row["active_valid"]),
                nmin=fmt(row["nearest"]["min"]), nm=fmt(row["nearest"]["mean"]), np=fmt(row["nearest"]["p95"]),
                dm=fmt(row["density"]["mean"], 5), dp=fmt(row["density"]["p95"], 5),
                cm=fmt(row["clutter"]["mean"], 1), cp=fmt(row["clutter"]["p95"], 1),
                planner=row["planner"], completion=fmt(row["completion"], 1)))
    lines += [
        "", "## 2. 资格边界", "",
        "- 第一档失败速度：{}。".format(fmt(result["first_failure_speed_mps"])),
        "- 当前最高稳定 qualification 速度：{}。".format(fmt(stable)),
        "- configured v_max 只表示规划约束；actual speed 使用 Observation C/FAST-LIO 因果速度统计，二者不互换。",
        "", "## 3. Observation C 与无效原因", "",
    ]
    for run in completed:
        csvm = run["csv_metrics"]
        lines.append("- {:.2f} m/s：invalid={}；unexplained={}；trajectory timestamp mismatch={}；kinematic gap={}；transition candidates={}。".format(
            run["fixed_v_max_mps"], json.dumps(csvm["invalid_reasons"], sort_keys=True),
            json.dumps(csvm["unexplained_invalid"], sort_keys=True),
            csvm["trajectory_timestamp_mismatch"], csvm["kinematic_gap"],
            run["transitions"].get("candidates")))
    lines += [
        "", "## 4. Planner、B-spline 与终态", "",
    ]
    for run in completed:
        chain = run["control_chain"]
        lines.append("- {:.2f} m/s：replan calls={}，random fallback inferred={}，CURRENT_POSITION episodes={}，B-spline sampled z=[{}, {}] m，tracking safety={}，collision proxy={}，emergency={}，final armed={}，landed_state={}。".format(
            run["fixed_v_max_mps"], chain["replan_calls"], chain["random_fallbacks_inferred"],
            chain["current_position_in_occupancy_episodes"],
            fmt(chain["bspline_sampled_min_z_m"]), fmt(chain["bspline_sampled_max_z_m"]),
            run["safety"].get("tracking_safety_terminal"),
            run["safety"].get("collision_proxy_terminal"), run["safety"].get("emergency_terminal"),
            chain["final_vehicle"]["armed"], chain["final_vehicle"]["landed_state"]))
        actual_acc = run["px4_ulog"].get("actual_acceleration_mps2", {})
        lines.append("  - pos_cmd speed mean/p95/max={}/{}/{} m/s；bridge raw={}/{}/{} m/s；pos_cmd acceleration={}/{}/{} m/s²；bridge raw acceleration={}/{}/{} m/s²；PX4 actual acceleration={}/{}/{} m/s²。".format(
            fmt(chain["position_command_speed_mps"]["mean"]),
            fmt(chain["position_command_speed_mps"]["p95"]),
            fmt(chain["position_command_speed_mps"]["max"]),
            fmt(chain["raw_setpoint_speed_mps"]["mean"]),
            fmt(chain["raw_setpoint_speed_mps"]["p95"]),
            fmt(chain["raw_setpoint_speed_mps"]["max"]),
            fmt(chain["position_command_acceleration_mps2"]["mean"]),
            fmt(chain["position_command_acceleration_mps2"]["p95"]),
            fmt(chain["position_command_acceleration_mps2"]["max"]),
            fmt(chain["raw_setpoint_acceleration_mps2"]["mean"]),
            fmt(chain["raw_setpoint_acceleration_mps2"]["p95"]),
            fmt(chain["raw_setpoint_acceleration_mps2"]["max"]),
            fmt(actual_acc.get("mean")), fmt(actual_acc.get("p95")),
            fmt(actual_acc.get("max"))))
        lines.append("  - bridge velocity saturation count/ratio={}/{}；bridge acceleration saturation count/ratio={}/{}。".format(
            chain["bridge_velocity_saturation_count"],
            fmt(chain["bridge_velocity_saturation_ratio"], 5),
            chain["bridge_acceleration_saturation_count"],
            fmt(chain["bridge_acceleration_saturation_ratio"], 5)))
    lines += [
        "", "## 5. 随速度变化趋势", "",
    ]
    if completed:
        configured = [run["fixed_v_max_mps"] for run in completed]
        actual_p95 = [run["csv_metrics"]["actual_speed_mps"]["p95"] for run in completed]
        tracking_p95 = [run["csv_metrics"]["tracking_error_m"]["p95"] for run in completed]
        ratios = [actual / configured if actual is not None else None
                  for actual, configured in zip(actual_p95, configured)]
        lines.extend([
            "- configured speed：{}。".format(configured),
            "- actual speed p95：{}；actual/configured p95 比例：{}。".format(
                [None if value is None else round(value, 4) for value in actual_p95],
                [None if value is None else round(value, 4) for value in ratios]),
            "- tracking p95：{}。趋势只描述已运行档，不外推 NOT_RUN 档。".format(
                [None if value is None else round(value, 4) for value in tracking_p95]),
        ])
    lines += [
        "", "## 6. 与 post-fix B 的可比边界", "",
        "新旧数据使用同一 Observation C、nearest/density/clutter 和 transition 语义；但 ceiling/max_acc/bridge generation 不同，因此只可做分层对照，不能混池为同一代 reward calibration。post-fix B 提供 0.30–1.75 m/s 低速参照，本轮提供 4.0/3.0 generation 的连续 qualification。",
        "", "| post-fix B speed | mission | actual p95 | tracking p95 | raw valid | active valid | nearest min | density mean | clutter mean |",
        "|---:|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for base in baseline_b:
        lines.append("| {speed:.2f} | {mission} | {actual} | {tracking} | {raw} | {active} | {nearest} | {density} | {clutter} |".format(
            speed=base["fixed_v_max_mps"], mission=base.get("mission_result"),
            actual=fmt(base.get("actual_speed", {}).get("p95")),
            tracking=fmt(base.get("tracking_error", {}).get("p95")),
            raw=fmt(base.get("raw_observation_valid_ratio")),
            active=fmt(base.get("training_active_valid_ratio")),
            nearest=fmt(base.get("nearest_obstacle", {}).get("min")),
            density=fmt(base.get("density", {}).get("mean"), 5),
            clutter=fmt(base.get("clutter", {}).get("mean"), 1)))
    lines += [
        "", "## 7. 对后续 Reward 设计的直接启示（不定义 Reward）", "",
        "- Reward 设计必须使用 actual speed/tracking，而不能使用 configured v_max 代替真实执行效果。",
        "- planner/collision/tracking terminal 必须保留独立 provenance，不能被速度收益抵消或重标为成功。",
        "- nearest、density、clutter 是上下文/诊断量；当前仍不是 clearance 硬保证，也未进入冻结 policy input。",
        "- Observation C unexplained invalid、timestamp mismatch 或 kinematic gap 若非零，应先修数据质量，不能以 Reward 权重掩盖。",
        "", "## 8. 是否进入 Reward 设计", "",
    ]
    all_requested_passed = len(completed) == len(SPEEDS) and all(run["pass"] for run in completed)
    data_clean = all(not run["csv_metrics"]["unexplained_invalid"]
                     and run["csv_metrics"]["trajectory_timestamp_mismatch"] == 0
                     and run["csv_metrics"]["kinematic_gap"] == 0 for run in completed)
    ready = all_requested_passed and data_clean
    lines.append("**{}**。{}".format(
        "YES" if ready else "NO",
        "全部速度档通过且关键 Observation C regression 为零，可冻结底层参数并转入独立 Reward 设计评审。"
        if ready else
        "存在未运行/失败速度档或数据质量阻断；保留当前底层参数与结果，不在本轮定义 Reward。"))
    Path(report_path).write_text("\n".join(lines) + "\n", encoding="utf-8")
    result["ready_for_reward_design"] = ready
    (root / "progressive_qualification_analysis.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path)
    parser.add_argument("--qualification-root", type=Path)
    parser.add_argument("--baseline-analysis", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.run_dir:
        result = analyze_run(args.run_dir)
    elif args.qualification_root and args.baseline_analysis and args.report:
        result = aggregate(args.qualification_root, args.baseline_analysis, args.report)
    else:
        raise SystemExit("use --run-dir or aggregate arguments")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
