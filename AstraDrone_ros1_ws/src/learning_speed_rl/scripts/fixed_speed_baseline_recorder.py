#!/usr/bin/env python3
"""Record one fixed-speed UAV1 run without publishing policy or control."""

import csv
import json
import math
import os
from collections import Counter

import numpy as np
import rospy
from astra_custom_msgs.msg import PlannerStatus
from geometry_msgs.msg import PoseStamped
from learning_speed_rl.msg import ObservationC
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String
from traj_utils.msg import Bspline


def _summary(values):
    finite = np.asarray([value for value in values if math.isfinite(value)], dtype=float)
    if finite.size == 0:
        return {"count": 0, "mean": None, "median": None, "p95": None, "max": None}
    return {
        "count": int(finite.size),
        "mean": float(np.mean(finite)),
        "median": float(np.median(finite)),
        "p95": float(np.percentile(finite, 95)),
        "max": float(np.max(finite)),
    }


class FixedSpeedBaselineRecorder:
    def __init__(self):
        self.run_id = str(rospy.get_param("~run_id"))
        self.output_dir = os.path.abspath(str(rospy.get_param("~output_dir")))
        self.requested_v_max = float(rospy.get_param("~requested_v_max"))
        self.clearance_rate_hz = float(rospy.get_param("~clearance_rate_hz", 2.0))
        if (
            not self.run_id
            or not math.isfinite(self.requested_v_max)
            or self.requested_v_max <= 0.0
            or not math.isfinite(self.clearance_rate_hz)
            or self.clearance_rate_hz <= 0.0
        ):
            raise rospy.ROSInitException("invalid fixed-speed baseline parameters")
        os.makedirs(self.output_dir, exist_ok=True)

        self.finalized = False
        self.run_start = rospy.Time.now().to_sec()
        self.mission_start = None
        self.mission_end = None
        self.mission_state = ""
        self.mission_success = False
        self.mission_failure = False
        self.mission_done = False
        self.orbit_complete = False
        self.latest_position = None
        self.last_raw_clearance_stamp = -math.inf
        self.last_inflated_clearance_stamp = -math.inf

        self.requested_samples = []
        self.filtered_samples = []
        self.applied_samples = []
        self.odom_speed_samples = []
        self.observation_rows = []
        self.observation_total = 0
        self.observation_valid = 0
        self.observation_invalid_reasons = Counter()
        self.observation_stamps = []
        self.observation_latencies = []
        self.tracking_norm = []
        self.tracking_xyz = [[], [], []]
        self.actual_speed = []
        self.actual_velocity_xyz = [[], [], []]
        self.raw_clearances = []
        self.inflated_clearances = []
        self.clearance_sample_counts = {"raw_filtered_points": 0, "ego_inflated_points": 0}

        self.trajectory_ids = []
        self.trajectory_id_set = set()
        self.pending_goal_stamp = None
        self.pending_goal_baseline_trajectory_ids = set()
        self.goal_to_trajectory_latencies = []
        self.planner_failure_events = 0
        self.planner_failure = False
        self.emergency_stop = False
        self.emergency_trajectory = False
        self.emergency_stop_timeout = False
        self.current_position_in_inflated_occupancy = False
        self.goal_in_inflated_occupancy = False
        self.last_planner_failure_key = None

        rospy.Subscriber("/uav1/tower_mission/state", String, self._state, queue_size=20)
        rospy.Subscriber("/uav1/tower_mission/mission_success", Bool, self._success, queue_size=2)
        rospy.Subscriber("/uav1/tower_mission/mission_failure", Bool, self._failure, queue_size=2)
        rospy.Subscriber("/uav1/tower_mission/mission_done", Bool, self._done, queue_size=2)
        rospy.Subscriber("/uav1/tower_mission/orbit_complete", Bool, self._orbit, queue_size=2)
        rospy.Subscriber("/uav1/Odometry", Odometry, self._odom, queue_size=20)
        rospy.Subscriber("/uav1/learning_speed/raw_v_max", Float64, self._raw_vmax, queue_size=20)
        rospy.Subscriber("/uav1/learning_speed/v_max", Float64, self._safe_vmax, queue_size=20)
        rospy.Subscriber("/uav1/learning_speed/applied_v_max", Float64, self._applied_vmax, queue_size=20)
        rospy.Subscriber("/uav1/learning_speed/observation_c", ObservationC, self._observation, queue_size=20)
        rospy.Subscriber("/uav1/planner/status", PlannerStatus, self._planner, queue_size=20)
        rospy.Subscriber("/uav1/planning/bspline", Bspline, self._trajectory, queue_size=20)
        rospy.Subscriber("/uav1/tower_mission/current_target", PoseStamped, self._target, queue_size=5)
        rospy.Subscriber("/uav1/stage3/cloud_registered_filtered", PointCloud2, self._raw_cloud, queue_size=1)
        rospy.Subscriber("/uav1/stage3/occupancy_inflate", PointCloud2, self._inflated_cloud, queue_size=1)
        rospy.on_shutdown(self.finalize)

    def _mission_active(self):
        return self.mission_start is not None and not self.mission_done

    def _state(self, message):
        now = rospy.Time.now().to_sec()
        self.mission_state = message.data
        if self.mission_start is None and message.data not in ("", "WAIT_INPUTS"):
            self.mission_start = now

    def _success(self, message):
        self.mission_success = bool(message.data)

    def _failure(self, message):
        self.mission_failure = bool(message.data)

    def _orbit(self, message):
        self.orbit_complete = bool(message.data)

    def _done(self, message):
        self.mission_done = bool(message.data)
        if self.mission_done and self.mission_end is None:
            self.mission_end = rospy.Time.now().to_sec()
            rospy.Timer(rospy.Duration(1.0), lambda _event: self.finalize(), oneshot=True)

    def _odom(self, message):
        p = message.pose.pose.position
        self.latest_position = np.asarray([p.x, p.y, p.z], dtype=float)
        if self._mission_active():
            v = message.twist.twist.linear
            speed = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            if math.isfinite(speed):
                self.odom_speed_samples.append(speed)

    def _raw_vmax(self, message):
        if self._mission_active() and math.isfinite(message.data):
            self.requested_samples.append(float(message.data))

    def _safe_vmax(self, message):
        if self._mission_active() and math.isfinite(message.data):
            self.filtered_samples.append(float(message.data))

    def _applied_vmax(self, message):
        if self._mission_active() and math.isfinite(message.data):
            self.applied_samples.append(float(message.data))

    def _observation(self, message):
        if not self._mission_active():
            return
        self.observation_total += 1
        stamp = message.header.stamp.to_sec()
        self.observation_stamps.append(stamp)
        latency = max(0.0, rospy.Time.now().to_sec() - stamp)
        self.observation_latencies.append(latency)
        if not message.valid:
            reasons = list(message.diagnostics) or ["unspecified_invalid"]
            self.observation_invalid_reasons.update(reasons)
            return
        self.observation_valid += 1
        error = [message.tracking_error_body.x, message.tracking_error_body.y,
                 message.tracking_error_body.z]
        velocity = [message.actual_velocity_body.x, message.actual_velocity_body.y,
                    message.actual_velocity_body.z]
        speed = math.sqrt(sum(value * value for value in velocity))
        self.tracking_norm.append(float(message.tracking_error_norm))
        self.actual_speed.append(speed)
        for axis in range(3):
            self.tracking_xyz[axis].append(float(error[axis]))
            self.actual_velocity_xyz[axis].append(float(velocity[axis]))
        self.observation_rows.append({
            "timestamp": stamp,
            "valid": 1,
            "tracking_error_x_body_m": error[0],
            "tracking_error_y_body_m": error[1],
            "tracking_error_z_body_m": error[2],
            "tracking_error_norm_m": float(message.tracking_error_norm),
            "actual_velocity_x_body_mps": velocity[0],
            "actual_velocity_y_body_mps": velocity[1],
            "actual_velocity_z_body_mps": velocity[2],
            "actual_speed_mps": speed,
            "previous_applied_v_max_mps": float(message.previous_v_max),
            "mission_state": self.mission_state,
            "trajectory_id": int(message.trajectory_id),
        })

    def _planner(self, message):
        if not self._mission_active():
            return
        failed = (not message.last_plan_success and message.failure_reason != PlannerStatus.NONE)
        key = (message.status_timestamp.to_nsec(), message.failure_reason,
               int(message.consecutive_plan_failures))
        if failed and key != self.last_planner_failure_key:
            self.planner_failure_events += 1
            self.last_planner_failure_key = key
        self.planner_failure = self.planner_failure or failed or message.consecutive_plan_failures > 0
        self.emergency_stop = self.emergency_stop or bool(message.emergency_stop_active)
        self.emergency_trajectory = self.emergency_trajectory or (
            bool(message.emergency_stop_active)
        )
        self.emergency_stop_timeout = self.emergency_stop_timeout or (
            message.failure_reason == PlannerStatus.EMERGENCY_STOP_TIMEOUT
        )
        self.current_position_in_inflated_occupancy = (
            self.current_position_in_inflated_occupancy
            or bool(message.current_position_in_collision)
        )
        self.goal_in_inflated_occupancy = (
            self.goal_in_inflated_occupancy or bool(message.goal_in_collision)
        )

    def _target(self, message):
        if self._mission_active():
            stamp = message.header.stamp.to_sec()
            self.pending_goal_stamp = stamp if stamp > 0.0 else rospy.Time.now().to_sec()
            self.pending_goal_baseline_trajectory_ids = set(self.trajectory_id_set)

    def _trajectory(self, message):
        if not self._mission_active():
            return
        trajectory_id = int(message.traj_id)
        if trajectory_id not in self.trajectory_id_set:
            self.trajectory_id_set.add(trajectory_id)
            self.trajectory_ids.append(trajectory_id)
        if (
            self.pending_goal_stamp is not None
            and trajectory_id not in self.pending_goal_baseline_trajectory_ids
        ):
            latency = rospy.Time.now().to_sec() - self.pending_goal_stamp
            if latency >= 0.0 and math.isfinite(latency):
                self.goal_to_trajectory_latencies.append(latency)
            self.pending_goal_stamp = None

    def _cloud_clearance(self, message, representation):
        if not self._mission_active() or self.latest_position is None:
            return
        stamp = message.header.stamp.to_sec()
        attribute = (
            "last_raw_clearance_stamp"
            if representation == "raw_filtered_points"
            else "last_inflated_clearance_stamp"
        )
        if stamp - getattr(self, attribute) < 1.0 / self.clearance_rate_hz:
            return
        setattr(self, attribute, stamp)
        minimum = math.inf
        try:
            for point in point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            ):
                distance = math.sqrt(
                    (point[0] - self.latest_position[0]) ** 2
                    + (point[1] - self.latest_position[1]) ** 2
                    + (point[2] - self.latest_position[2]) ** 2
                )
                minimum = min(minimum, distance)
        except (KeyError, ValueError, TypeError) as error:
            rospy.logwarn_throttle(1.0, "baseline clearance decode failed: %s", error)
            return
        if math.isfinite(minimum):
            target = self.raw_clearances if representation == "raw_filtered_points" else self.inflated_clearances
            target.append(minimum)
            self.clearance_sample_counts[representation] += 1

    def _raw_cloud(self, message):
        self._cloud_clearance(message, "raw_filtered_points")

    def _inflated_cloud(self, message):
        self._cloud_clearance(message, "ego_inflated_points")

    def _observation_rate(self):
        if len(self.observation_stamps) < 2:
            return None
        duration = max(self.observation_stamps) - min(self.observation_stamps)
        return None if duration <= 0.0 else (len(self.observation_stamps) - 1) / duration

    def finalize(self):
        if self.finalized:
            return
        self.finalized = True
        completion_time = None
        if self.mission_start is not None and self.mission_end is not None:
            completion_time = max(0.0, self.mission_end - self.mission_start)
        outcome = "success" if self.mission_success else (
            "failure" if self.mission_failure or self.mission_done else "incomplete"
        )
        summary = {
            "schema_version": "fixed_speed_baseline.v1",
            "run_id": self.run_id,
            "requested_fixed_v_max_mps": self.requested_v_max,
            "mission": {
                "success": self.mission_success,
                "failure": self.mission_failure,
                "done": self.mission_done,
                "orbit_complete": self.orbit_complete,
                "terminal_state": self.mission_state,
                "outcome": outcome,
                "completion_time_sec": completion_time,
            },
            "safety": {
                "minimum_raw_filtered_point_distance_m": min(self.raw_clearances) if self.raw_clearances else None,
                "minimum_ego_inflated_occupied_center_distance_m": min(self.inflated_clearances) if self.inflated_clearances else None,
                "raw_clearance_representation": "UAV center to nearest point in stage3/cloud_registered_filtered; no inflation or vehicle radius subtraction",
                "inflated_clearance_representation": "UAV center to nearest occupied voxel center in EGO occupancy_inflate; already inflated and not physical surface clearance",
                "clearance_sampling_rate_hz": self.clearance_rate_hz,
                "clearance_sample_counts": self.clearance_sample_counts,
                "collision": self.current_position_in_inflated_occupancy,
                "collision_representation": "PlannerStatus.current_position_in_collision against EGO inflated occupancy; physical Gazebo contact is not instrumented",
                "goal_in_inflated_occupancy": self.goal_in_inflated_occupancy,
                "emergency_stop": self.emergency_stop,
            },
            "tracking": {
                "tracking_error_norm_m": _summary(self.tracking_norm),
                "tracking_error_body_x_m": _summary(self.tracking_xyz[0]),
                "tracking_error_body_y_m": _summary(self.tracking_xyz[1]),
                "tracking_error_body_z_m": _summary(self.tracking_xyz[2]),
            },
            "velocity": {
                "requested_source_mps": _summary(self.requested_samples),
                "filtered_v_max_mps": _summary(self.filtered_samples),
                "applied_v_max_mps": _summary(self.applied_samples),
                "actual_speed_observation_c_mps": _summary(self.actual_speed),
                "actual_speed_odom_mps": _summary(self.odom_speed_samples),
                "actual_velocity_body_x_mps": _summary(self.actual_velocity_xyz[0]),
                "actual_velocity_body_y_mps": _summary(self.actual_velocity_xyz[1]),
                "actual_velocity_body_z_mps": _summary(self.actual_velocity_xyz[2]),
            },
            "planner": {
                "ego_replanning_count": max(0, len(self.trajectory_ids) - 1),
                "trajectory_replacement_count": max(0, len(self.trajectory_ids) - 1),
                "unique_trajectory_count": len(self.trajectory_ids),
                "goal_to_first_trajectory_latency_sec": _summary(self.goal_to_trajectory_latencies),
                "latency_definition": "mission current_target callback to first new Bspline callback; EGO internal optimizer latency is not exposed",
                "planner_failure": self.planner_failure,
                "planner_failure_events": self.planner_failure_events,
                "emergency_trajectory": self.emergency_trajectory,
                "emergency_trajectory_representation": "PlannerStatus.emergency_stop_active, i.e. EGO EMERGENCY_STOP state; no separate trajectory-class topic is exposed",
                "emergency_stop_timeout": self.emergency_stop_timeout,
            },
            "observation_c": {
                "messages": self.observation_total,
                "valid": self.observation_valid,
                "valid_ratio": (float(self.observation_valid) / self.observation_total if self.observation_total else None),
                "invalid_reason_counts": dict(self.observation_invalid_reasons),
                "update_rate_hz": self._observation_rate(),
                "latency_sec": _summary(self.observation_latencies),
            },
        }
        with open(os.path.join(self.output_dir, "run_summary.json"), "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
        fieldnames = [
            "timestamp", "valid", "tracking_error_x_body_m",
            "tracking_error_y_body_m", "tracking_error_z_body_m",
            "tracking_error_norm_m", "actual_velocity_x_body_mps",
            "actual_velocity_y_body_mps", "actual_velocity_z_body_mps",
            "actual_speed_mps", "previous_applied_v_max_mps", "mission_state",
            "trajectory_id", "next_outcome",
        ]
        with open(os.path.join(self.output_dir, "observation_samples.csv"), "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.observation_rows:
                output = dict(row)
                output["next_outcome"] = outcome
                writer.writerow(output)


def main():
    rospy.init_node("fixed_speed_baseline_recorder")
    FixedSpeedBaselineRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
