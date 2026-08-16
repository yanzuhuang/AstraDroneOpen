#!/usr/bin/env python3
"""Aggregate immutable per-run JSON into baseline CSV/JSON artifacts."""

import argparse
import csv
import glob
import json
import os
from collections import defaultdict

import numpy as np


def _get(item, path, default=None):
    for key in path.split("."):
        if not isinstance(item, dict) or key not in item:
            return default
        item = item[key]
    return item


def _finite(values):
    return np.asarray([value for value in values if isinstance(value, (int, float)) and np.isfinite(value)], dtype=float)


def _aggregate(values):
    values = _finite(values)
    if values.size == 0:
        return {"mean": None, "median": None, "p95": None, "max": None}
    return {
        "mean": float(np.mean(values)),
        "median": float(np.median(values)),
        "p95": float(np.percentile(values, 95)),
        "max": float(np.max(values)),
    }


RUN_COLUMNS = [
    "run_id", "valid_for_aggregation", "v_max_mps", "mission_success", "mission_failure",
    "mission_done", "orbit_complete", "completion_time_sec",
    "min_raw_clearance_m", "min_inflated_center_distance_m", "collision",
    "non_return_collision_proxy", "collision_proxy_mission_states",
    "emergency_stop", "tracking_mean_m", "tracking_median_m",
    "tracking_p95_m", "tracking_max_m", "actual_speed_mean_mps",
    "actual_speed_median_mps", "actual_speed_p95_mps", "actual_speed_max_mps",
    "tracking_body_x_mean_m", "tracking_body_x_median_m",
    "tracking_body_x_p95_m", "tracking_body_x_max_m",
    "tracking_body_y_mean_m", "tracking_body_y_median_m",
    "tracking_body_y_p95_m", "tracking_body_y_max_m",
    "tracking_body_z_mean_m", "tracking_body_z_median_m",
    "tracking_body_z_p95_m", "tracking_body_z_max_m",
    "applied_v_max_mean_mps", "ego_replanning_count",
    "trajectory_replacement_count",
    "planning_latency_mean_sec", "planning_latency_p95_sec",
    "planner_failure", "emergency_trajectory", "observation_c_valid_ratio",
    "observation_c_rate_hz", "observation_c_latency_p95_sec",
]


def flatten_run(item):
    collision_by_state = _get(
        item, "safety.current_inflated_occupancy_samples_by_mission_state", {}
    )
    non_return_collision = any(
        int(count) > 0 and state not in (
            "NORMAL_RETURN", "RETURN_HOME", "LANDING", "DONE", "ERROR"
        )
        for state, count in collision_by_state.items()
    )
    return {
        "run_id": item["run_id"],
        # Only a latched terminal mission result is a baseline trial.  A ROS,
        # simulator or orchestration interruption must remain visible in the
        # run ledger but must not dilute mission success/failure statistics.
        "valid_for_aggregation": bool(_get(item, "mission.done", False)),
        "v_max_mps": item["requested_fixed_v_max_mps"],
        "mission_success": _get(item, "mission.success"),
        "mission_failure": _get(item, "mission.failure"),
        "mission_done": _get(item, "mission.done"),
        "orbit_complete": _get(item, "mission.orbit_complete"),
        "completion_time_sec": _get(item, "mission.completion_time_sec"),
        "min_raw_clearance_m": _get(item, "safety.minimum_raw_filtered_point_distance_m"),
        "min_inflated_center_distance_m": _get(item, "safety.minimum_ego_inflated_occupied_center_distance_m"),
        "collision": _get(item, "safety.collision"),
        "non_return_collision_proxy": non_return_collision,
        "collision_proxy_mission_states": ";".join(
            sorted(state for state, count in collision_by_state.items()
                   if int(count) > 0)
        ),
        "emergency_stop": _get(item, "safety.emergency_stop"),
        "tracking_mean_m": _get(item, "tracking.tracking_error_norm_m.mean"),
        "tracking_median_m": _get(item, "tracking.tracking_error_norm_m.median"),
        "tracking_p95_m": _get(item, "tracking.tracking_error_norm_m.p95"),
        "tracking_max_m": _get(item, "tracking.tracking_error_norm_m.max"),
        "actual_speed_mean_mps": _get(item, "velocity.actual_speed_observation_c_mps.mean"),
        "actual_speed_median_mps": _get(item, "velocity.actual_speed_observation_c_mps.median"),
        "actual_speed_p95_mps": _get(item, "velocity.actual_speed_observation_c_mps.p95"),
        "actual_speed_max_mps": _get(item, "velocity.actual_speed_observation_c_mps.max"),
        "tracking_body_x_mean_m": _get(item, "tracking.tracking_error_body_x_m.mean"),
        "tracking_body_x_median_m": _get(item, "tracking.tracking_error_body_x_m.median"),
        "tracking_body_x_p95_m": _get(item, "tracking.tracking_error_body_x_m.p95"),
        "tracking_body_x_max_m": _get(item, "tracking.tracking_error_body_x_m.max"),
        "tracking_body_y_mean_m": _get(item, "tracking.tracking_error_body_y_m.mean"),
        "tracking_body_y_median_m": _get(item, "tracking.tracking_error_body_y_m.median"),
        "tracking_body_y_p95_m": _get(item, "tracking.tracking_error_body_y_m.p95"),
        "tracking_body_y_max_m": _get(item, "tracking.tracking_error_body_y_m.max"),
        "tracking_body_z_mean_m": _get(item, "tracking.tracking_error_body_z_m.mean"),
        "tracking_body_z_median_m": _get(item, "tracking.tracking_error_body_z_m.median"),
        "tracking_body_z_p95_m": _get(item, "tracking.tracking_error_body_z_m.p95"),
        "tracking_body_z_max_m": _get(item, "tracking.tracking_error_body_z_m.max"),
        "applied_v_max_mean_mps": _get(item, "velocity.applied_v_max_mps.mean"),
        "ego_replanning_count": _get(item, "planner.ego_replanning_count"),
        "trajectory_replacement_count": _get(item, "planner.trajectory_replacement_count"),
        "planning_latency_mean_sec": _get(item, "planner.goal_to_first_trajectory_latency_sec.mean"),
        "planning_latency_p95_sec": _get(item, "planner.goal_to_first_trajectory_latency_sec.p95"),
        "planner_failure": _get(item, "planner.planner_failure"),
        "emergency_trajectory": _get(item, "planner.emergency_trajectory"),
        "observation_c_valid_ratio": _get(item, "observation_c.valid_ratio"),
        "observation_c_rate_hz": _get(item, "observation_c.update_rate_hz"),
        "observation_c_latency_p95_sec": _get(item, "observation_c.latency_sec.p95"),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root")
    args = parser.parse_args()
    root = os.path.abspath(args.root)
    paths = sorted(glob.glob(os.path.join(root, "runs", "*", "run_summary.json")))
    runs = []
    raw = []
    for path in paths:
        with open(path, encoding="utf-8") as stream:
            item = json.load(stream)
        raw.append(item)
        runs.append(flatten_run(item))
    os.makedirs(root, exist_ok=True)
    with open(os.path.join(root, "baseline_runs.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=RUN_COLUMNS)
        writer.writeheader()
        writer.writerows(runs)

    grouped = defaultdict(list)
    for run in runs:
        if not run["valid_for_aggregation"]:
            continue
        grouped[float(run["v_max_mps"])].append(run)
    summary_rows = []
    summary_json = {
        "schema_version": "fixed_speed_baseline_summary.v1",
        "valid_run_count": sum(run["valid_for_aggregation"] for run in runs),
        "excluded_nonterminal_run_ids": [
            run["run_id"] for run in runs if not run["valid_for_aggregation"]
        ],
        "speeds": [],
    }
    for speed in sorted(grouped):
        group = grouped[speed]
        successful = [run for run in group if run["mission_success"]]
        row = {
            "v_max_mps": speed,
            "runs": len(group),
            "success_rate": sum(bool(run["mission_success"]) for run in group) / len(group),
            "failure_rate": sum(bool(run["mission_failure"]) for run in group) / len(group),
            "orbit_complete_rate": sum(bool(run["orbit_complete"]) for run in group) / len(group),
            "terminal_duration_mean_sec": _aggregate(
                [run["completion_time_sec"] for run in group]
            )["mean"],
            "terminal_duration_p95_sec": _aggregate(
                [run["completion_time_sec"] for run in group]
            )["p95"],
            "successful_completion_time_mean_sec": _aggregate(
                [run["completion_time_sec"] for run in successful]
            )["mean"],
            "successful_completion_time_p95_sec": _aggregate(
                [run["completion_time_sec"] for run in successful]
            )["p95"],
            "min_raw_clearance_m": min(_finite([run["min_raw_clearance_m"] for run in group]), default=None),
            "min_inflated_center_distance_m": min(_finite([run["min_inflated_center_distance_m"] for run in group]), default=None),
            "collision_proxy_runs": sum(bool(run["collision"]) for run in group),
            "non_return_collision_proxy_runs": sum(
                bool(run["non_return_collision_proxy"]) for run in group
            ),
            "tracking_mean_mean_m": _aggregate([run["tracking_mean_m"] for run in group])["mean"],
            "tracking_median_mean_m": _aggregate([run["tracking_median_m"] for run in group])["mean"],
            "tracking_p95_mean_m": _aggregate([run["tracking_p95_m"] for run in group])["mean"],
            "tracking_max_m": _aggregate([run["tracking_max_m"] for run in group])["max"],
            "actual_speed_mean_mean_mps": _aggregate([run["actual_speed_mean_mps"] for run in group])["mean"],
            "actual_speed_median_mean_mps": _aggregate([run["actual_speed_median_mps"] for run in group])["mean"],
            "actual_speed_p95_mean_mps": _aggregate([run["actual_speed_p95_mps"] for run in group])["mean"],
            "actual_speed_max_mps": _aggregate([run["actual_speed_max_mps"] for run in group])["max"],
            "applied_v_max_mean_mps": _aggregate([run["applied_v_max_mean_mps"] for run in group])["mean"],
            "ego_replanning_count_mean": _aggregate([run["ego_replanning_count"] for run in group])["mean"],
            "trajectory_replacement_count_mean": _aggregate([run["trajectory_replacement_count"] for run in group])["mean"],
            "planning_latency_mean_mean_sec": _aggregate([run["planning_latency_mean_sec"] for run in group])["mean"],
            "planning_latency_p95_mean_sec": _aggregate([run["planning_latency_p95_sec"] for run in group])["mean"],
            "planner_failure_runs": sum(bool(run["planner_failure"]) for run in group),
            "emergency_runs": sum(bool(run["emergency_stop"] or run["emergency_trajectory"]) for run in group),
            "observation_c_valid_ratio_mean": _aggregate([run["observation_c_valid_ratio"] for run in group])["mean"],
            "observation_c_rate_mean_hz": _aggregate([run["observation_c_rate_hz"] for run in group])["mean"],
            "observation_c_latency_p95_mean_sec": _aggregate([run["observation_c_latency_p95_sec"] for run in group])["mean"],
        }
        applied = row["applied_v_max_mean_mps"]
        row["applied_minus_requested_mean_mps"] = (
            applied - speed if applied is not None else None
        )
        row["actual_mean_to_applied_ratio"] = (
            row["actual_speed_mean_mean_mps"] / applied
            if applied and row["actual_speed_mean_mean_mps"] is not None else None
        )
        row["actual_p95_to_applied_ratio"] = (
            row["actual_speed_p95_mean_mps"] / applied
            if applied and row["actual_speed_p95_mean_mps"] is not None else None
        )
        body_axes = {}
        for axis in ("x", "y", "z"):
            body_axes[axis] = {
                "mean_of_run_means_m": _aggregate(
                    [run["tracking_body_{}_mean_m".format(axis)] for run in group]
                )["mean"],
                "mean_of_run_medians_m": _aggregate(
                    [run["tracking_body_{}_median_m".format(axis)] for run in group]
                )["mean"],
                "mean_of_run_p95_m": _aggregate(
                    [run["tracking_body_{}_p95_m".format(axis)] for run in group]
                )["mean"],
                "maximum_over_runs_m": _aggregate(
                    [run["tracking_body_{}_max_m".format(axis)] for run in group]
                )["max"],
            }
        invalid_reasons = defaultdict(int)
        for item in raw:
            if (not bool(_get(item, "mission.done", False)) or
                    float(item["requested_fixed_v_max_mps"]) != speed):
                continue
            for reason, count in _get(
                    item, "observation_c.invalid_reason_counts", {}).items():
                invalid_reasons[reason] += int(count)
        summary_rows.append(row)
        json_row = dict(row)
        json_row["tracking_body_xyz"] = body_axes
        json_row["observation_c_invalid_reason_counts"] = dict(
            sorted(invalid_reasons.items())
        )
        summary_json["speeds"].append(json_row)
    columns = list(summary_rows[0].keys()) if summary_rows else ["v_max_mps", "runs"]
    with open(os.path.join(root, "baseline_summary.csv"), "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        writer.writerows(summary_rows)
    with open(os.path.join(root, "baseline_summary.json"), "w", encoding="utf-8") as stream:
        json.dump(summary_json, stream, indent=2, sort_keys=True)
        stream.write("\n")

    contract = {
        "scope": "field availability audit only; no reward or reward weights",
        "fields": [
            {"field": "timestamp_t", "source": "/uav1/learning_speed/observation_c.header.stamp", "source_code": "learning_speed_rl/scripts/observation_c_node.py", "frame": "uav1/body packet with trajectory source uav1/camera_init", "timestamp": "ObservationC.header.stamp (lidar acquisition ROS time)", "unit": "s ROS time", "update_rate": "measured per run", "valid_condition": "ObservationC.valid"},
            {"field": "lidar_surrogate", "source": "/uav1/learning_speed/observation_c.lidar_*", "source_code": "learning_speed_rl/observation/v2 and observation_c_node.py", "frame": "uav1/body", "timestamp": "ObservationC.header.stamp inherited from lidar surrogate", "unit": "normalized range plus masks/semantic enum", "update_rate": "Observation C rate", "valid_condition": "ObservationC.valid and lidar masks valid"},
            {"field": "ego_future_trajectory", "source": "/uav1/learning_speed/observation_c.future_positions_body", "source_code": "learning_speed_rl/observation/scheme_c and observation_c_node.py", "frame": "uav1/body at t", "timestamp": "ObservationC.header.stamp, with trajectory_start_time also retained", "unit": "m", "update_rate": "Observation C rate", "valid_condition": "active monotonic Bspline covering t"},
            {"field": "actual_velocity", "source": "/uav1/learning_speed/observation_c.actual_velocity_body", "source_code": "learning_speed_rl/observation/scheme_c and observation_c_node.py", "frame": "uav1/body", "timestamp": "timestamp-aligned to ObservationC.header.stamp", "unit": "m/s", "update_rate": "Observation C rate", "valid_condition": "timestamped odometry finite-difference state available"},
            {"field": "tracking_error", "source": "/uav1/learning_speed/observation_c.tracking_error_body,norm", "source_code": "learning_speed_rl/observation/scheme_c and observation_c_node.py", "frame": "uav1/body", "timestamp": "timestamp-aligned to ObservationC.header.stamp", "unit": "m", "update_rate": "Observation C rate", "valid_condition": "active Bspline and interpolated pose at t"},
            {"field": "previous_applied_v_max", "source": "/uav1/learning_speed/applied_v_max joined into ObservationC.previous_v_max", "source_code": "ego_replan_fsm.cpp dynamic speed acknowledgement plus observation_c_node.py", "frame": "scalar", "timestamp": "adapter receipt ROS time; causal latest sample <= ObservationC.header.stamp", "unit": "m/s", "update_rate": "adapter/EGO acknowledgement plus Observation C sampling", "valid_condition": "latest acknowledgement timestamp <= t and within configured range"},
            {"field": "mission_state", "source": "/uav1/tower_mission/state", "source_code": "astra_tower_mission/src/sector_inspection_mission_node.cpp", "frame": "state enum", "timestamp": "subscriber receipt ROS time; std_msgs/String has no header", "unit": "none", "update_rate": "latched on transition", "valid_condition": "nonempty known state"},
            {"field": "planner_state", "source": "/uav1/planner/status", "source_code": "plan_manage/src/ego_replan_fsm.cpp and astra_custom_msgs/PlannerStatus.msg", "frame": "uav1/camera_init in header", "timestamp": "PlannerStatus.status_timestamp and header.stamp", "unit": "state/counters", "update_rate": "10 Hz configured", "valid_condition": "fresh PlannerStatus timestamp"},
            {"field": "raw_clearance", "source": "/uav1/stage3/cloud_registered_filtered plus /uav1/Odometry", "source_code": "learning_speed_rl/scripts/fixed_speed_baseline_recorder.py", "frame": "uav1/camera_init", "timestamp": "cloud header.stamp with latest received odometry (not exact-time synchronized)", "unit": "m center-to-point", "update_rate": "2 Hz recorder sample", "valid_condition": "fresh same-frame cloud; baseline metric only, not yet a training-step field"},
            {"field": "inflated_clearance", "source": "/uav1/stage3/occupancy_inflate plus /uav1/Odometry", "source_code": "learning_speed_rl/scripts/fixed_speed_baseline_recorder.py", "frame": "uav1/camera_init", "timestamp": "cloud header.stamp with latest received odometry (not exact-time synchronized)", "unit": "m center-to-inflated-voxel-center", "update_rate": "2 Hz recorder sample", "valid_condition": "fresh same-frame cloud; baseline metric only; never combine with raw clearance"},
            {"field": "next_outcome", "source": "/uav1/tower_mission/mission_success,mission_failure,mission_done,orbit_complete post-run join", "source_code": "astra_tower_mission/mission_completion.h and fixed_speed_baseline_recorder.py", "frame": "episode", "timestamp": "terminal subscriber receipt ROS time; Bool topics have no header", "unit": "categorical", "update_rate": "terminal", "valid_condition": "mission_done true"},
        ],
        "not_reliably_available": [
            "EGO internal optimizer-only planning latency; current metric is goal-to-first-Bspline callback latency",
            "physical vehicle-surface clearance; current raw metric is center-to-point-return and inflated metric is center-to-voxel-center",
            "physical Gazebo contact collision; PlannerStatus only exposes current/goal occupancy against the EGO inflated map",
            "training-step clearance aligned at timestamp t; baseline recorder currently joins each cloud to latest odometry rather than exact-time interpolation",
            "causal next outcome at each step before a horizon/termination definition is reviewed",
        ],
    }
    with open(os.path.join(root, "training_data_contract_audit.json"), "w", encoding="utf-8") as stream:
        json.dump(contract, stream, indent=2, sort_keys=True)
        stream.write("\n")


if __name__ == "__main__":
    main()
