#!/usr/bin/env python3
"""Read-only recorder for manual Learning Speed reward calibration.

The node has no publishers and never computes a reward.  It records diagnostic
metrics and causally valid transition candidates under runtime_artifacts/.
"""

import csv
import json
import math
import os
import threading

import numpy as np
import rospy
from astra_custom_msgs.msg import PlannerStatus
from learning_speed_rl.msg import ObservationC, SpeedActionStamped
from learning_speed_rl.training import (
    AppliedSpeedAction,
    OfficialTrajectoryIdentity,
    PlannerFailureEpisodeTracker,
    PolicyStateProvenance,
    PolicyStateV1,
    SacTransitionV1,
    TrackingSafetyMirror,
    lidar_clutter_metrics,
    validate_artifact_root,
)
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String
from traj_utils.msg import Bspline


SAMPLE_FIELDS = (
    "observation_stamp_sec",
    "observation_receive_sec",
    "observation_latency_sec",
    "observation_version",
    "observation_valid",
    "observation_invalid_reasons",
    "observation_frame",
    "trajectory_id",
    "trajectory_start_time_sec",
    "trajectory_source_frame",
    "nearest_obstacle_distance_m",
    "known_obstacle_bin_count",
    "known_obstacle_bin_fraction",
    "observed_free_bin_count",
    "unknown_bin_count",
    "raw_filtered_nearest_point_distance_m",
    "raw_filtered_clearance_stamp_sec",
    "inflated_occupied_center_distance_m",
    "inflated_clearance_stamp_sec",
    "actual_speed_mps",
    "actual_velocity_x_body_mps",
    "actual_velocity_y_body_mps",
    "actual_velocity_z_body_mps",
    "tracking_error_x_body_m",
    "tracking_error_y_body_m",
    "tracking_error_z_body_m",
    "tracking_error_norm_m",
    "previous_applied_v_max_mps",
    "latest_requested_v_max_mps",
    "latest_requested_stamp_sec",
    "latest_filtered_v_max_mps",
    "latest_filtered_stamp_sec",
    "latest_action_receive_sec",
    "latest_applied_v_max_mps",
    "latest_applied_receive_sec",
    "mission_state",
    "planner_state",
    "planner_failure_active",
    "planner_failure_reason",
    "planner_failure_episode_id",
    "planner_consecutive_failures",
    "collision_safety_terminal",
    "emergency_safety_terminal",
    "tracking_safety_terminal",
    "dangerous_terminal",
    "mission_success",
    "mission_failure",
    "mission_done",
)


class CalibrationDataRecorder:
    def __init__(self):
        self._lock = threading.RLock()
        self._run_id = str(rospy.get_param("~run_id", "manual")).strip()
        output_dir = str(rospy.get_param("~output_dir", "")).strip()
        if not self._run_id or not output_dir:
            raise rospy.ROSInitException("run_id and output_dir are required")
        self._output_dir = validate_artifact_root(output_dir)
        if self._output_dir.exists() and any(self._output_dir.iterdir()):
            raise rospy.ROSInitException(
                "refusing to overwrite non-empty calibration output: {}".format(
                    self._output_dir
                )
            )
        self._output_dir.mkdir(parents=True, exist_ok=True)

        tracking_limit = float(rospy.get_param("~tracking_safety/limit_m", 1.0))
        tracking_duration = float(
            rospy.get_param("~tracking_safety/duration_sec", 1.0)
        )
        self._tracking_mirror = TrackingSafetyMirror(
            tracking_limit, tracking_duration
        )
        self._clearance_rate_hz = float(
            rospy.get_param("~clearance_sampling_rate_hz", 2.0)
        )
        self._applied_pair_tolerance = float(
            rospy.get_param("~applied_pair_tolerance_mps", 0.005)
        )
        if not math.isfinite(self._clearance_rate_hz) or self._clearance_rate_hz <= 0.0:
            raise rospy.ROSInitException("clearance_sampling_rate_hz must be positive")
        if (
            not math.isfinite(self._applied_pair_tolerance)
            or self._applied_pair_tolerance < 0.0
        ):
            raise rospy.ROSInitException("applied_pair_tolerance_mps is invalid")
        self._planner_episodes = PlannerFailureEpisodeTracker()
        self._finalized = False
        self._shutdown_scheduled = False
        self._started_sec = rospy.Time.now().to_sec()
        self._ended_sec = None

        self._mission_state = ""
        self._mission_success = False
        self._mission_failure = False
        self._mission_done = False
        self._planner_state = ""
        self._planner_failure_active = False
        self._planner_failure_reason = ""
        self._planner_consecutive_failures = 0
        self._bridge_state = ""
        self._collision_terminal = False
        self._emergency_terminal = False

        self._latest_requested = None
        self._latest_filtered = None
        self._latest_action_receive_sec = None
        self._latest_applied = None
        self._latest_official_trajectory = None
        self._official_trajectory_history = []
        self._latest_position_world = None
        self._latest_odom_frame = ""
        self._latest_raw_clearance = None
        self._latest_inflated_clearance = None
        self._last_clearance_stamp = {"raw": -math.inf, "inflated": -math.inf}
        self._latest_state = None
        self._pending_transition = None
        self._last_state_key_used = None
        self._transition_terminal_written = False

        self._observation_messages = 0
        self._valid_observations = 0
        self._transition_candidates = 0
        self._skipped_action_no_state = 0
        self._skipped_action_stale_trajectory = 0
        self._skipped_action_pending = 0
        self._dropped_incomplete_transitions = 0
        self._ignored_unmatched_applied = 0

        sample_path = self._output_dir / "calibration_samples.csv"
        transition_path = self._output_dir / "transition_candidates.jsonl"
        self._sample_stream = sample_path.open("w", newline="", encoding="utf-8")
        self._sample_writer = csv.DictWriter(
            self._sample_stream, fieldnames=SAMPLE_FIELDS
        )
        self._sample_writer.writeheader()
        self._transition_stream = transition_path.open("w", encoding="utf-8")
        diagnostics_path = self._output_dir / "observation_diagnostics.jsonl"
        self._diagnostics_stream = diagnostics_path.open("w", encoding="utf-8")

        topic = lambda name, default: str(rospy.get_param("~topics/" + name, default))
        rospy.Subscriber(
            topic("observation_c", "learning_speed/observation_c"),
            ObservationC,
            self._observation,
            queue_size=50,
        )
        rospy.Subscriber(
            topic("official_trajectory", "planning/bspline"),
            Bspline,
            self._trajectory,
            queue_size=20,
        )
        rospy.Subscriber(
            topic("action_stamped", "learning_speed/action_stamped"),
            SpeedActionStamped,
            self._action,
            queue_size=50,
        )
        rospy.Subscriber(
            topic("applied_v_max", "learning_speed/applied_v_max"),
            Float64,
            self._applied,
            queue_size=50,
        )
        rospy.Subscriber(
            topic("planner_status", "planner/status"),
            PlannerStatus,
            self._planner,
            queue_size=50,
        )
        rospy.Subscriber(
            topic("mission_state", "tower_mission/state"),
            String,
            self._state,
            queue_size=20,
        )
        rospy.Subscriber(
            topic("mission_success", "tower_mission/mission_success"),
            Bool,
            self._success,
            queue_size=5,
        )
        rospy.Subscriber(
            topic("mission_failure", "tower_mission/mission_failure"),
            Bool,
            self._failure,
            queue_size=5,
        )
        rospy.Subscriber(
            topic("mission_done", "tower_mission/mission_done"),
            Bool,
            self._done,
            queue_size=5,
        )
        rospy.Subscriber(
            topic("bridge_state", "ego_mavros_bridge/state"),
            String,
            self._bridge,
            queue_size=20,
        )
        rospy.Subscriber(
            topic("bridge_tracking_error", "ego_mavros_bridge/tracking_error"),
            Float64,
            self._tracking_error,
            queue_size=50,
        )
        rospy.Subscriber(
            topic("odom", "Odometry"), Odometry, self._odom, queue_size=50
        )
        rospy.Subscriber(
            topic("raw_filtered_cloud", "stage3/cloud_registered_filtered"),
            PointCloud2,
            lambda message: self._cloud_clearance(message, "raw"),
            queue_size=1,
        )
        rospy.Subscriber(
            topic("inflated_occupancy", "stage3/occupancy_inflate"),
            PointCloud2,
            lambda message: self._cloud_clearance(message, "inflated"),
            queue_size=1,
        )
        rospy.on_shutdown(self.finalize)
        rospy.logwarn(
            "Learning Speed calibration recorder is read-only; output=%s; reward/SAC are disabled",
            self._output_dir,
        )

    @staticmethod
    def _now():
        return rospy.Time.now().to_sec()

    @staticmethod
    def _finite_speed(message):
        value = float(message.data)
        return value if math.isfinite(value) and value > 0.0 else None

    @staticmethod
    def _trajectory_identity_from_bspline(message):
        return OfficialTrajectoryIdentity(
            trajectory_id=int(message.traj_id),
            start_time_sec=message.start_time.to_sec(),
            source_frame=message.frame_id.lstrip("/"),
        )

    @staticmethod
    def _trajectory_identity_from_observation(message):
        return OfficialTrajectoryIdentity(
            trajectory_id=int(message.trajectory_id),
            start_time_sec=message.trajectory_start_time.to_sec(),
            source_frame=message.trajectory_source_frame.lstrip("/"),
        )

    def _trajectory(self, message):
        try:
            identity = self._trajectory_identity_from_bspline(message)
        except ValueError:
            return
        with self._lock:
            if self._finalized:
                return
            current = self._latest_official_trajectory
            if current is None or (
                identity.start_time_sec,
                identity.trajectory_id,
            ) >= (current.start_time_sec, current.trajectory_id):
                self._latest_official_trajectory = identity
                self._official_trajectory_history.append((self._now(), identity))
                self._official_trajectory_history = self._official_trajectory_history[-100:]

    def _action(self, message):
        requested_value = float(message.requested_v_max)
        filtered_value = float(message.filtered_v_max)
        action_stamp = message.header.stamp.to_sec()
        if (
            message.version != "learning_speed_action_v1.0"
            or not math.isfinite(requested_value)
            or requested_value <= 0.0
            or not math.isfinite(filtered_value)
            or filtered_value <= 0.0
            or action_stamp <= 0.0
        ):
            return
        now = self._now()
        with self._lock:
            if self._finalized:
                return
            self._latest_requested = (action_stamp, requested_value)
            self._latest_filtered = (action_stamp, filtered_value)
            self._latest_action_receive_sec = now
            if (
                self._transition_terminal_written
                or self._dangerous_terminal()
                or self._mission_done
            ):
                return
            if self._pending_transition is not None:
                self._skipped_action_pending += 1
                return
            if self._latest_state is None:
                self._skipped_action_no_state += 1
                return
            state_key = (
                self._latest_state.provenance.observation_stamp_sec,
                self._latest_state.provenance.official_trajectory,
            )
            if state_key == self._last_state_key_used:
                self._skipped_action_no_state += 1
                return
            available = [
                identity
                for receive_sec, identity in self._official_trajectory_history
                if receive_sec <= action_stamp + 1.0e-9
            ]
            latest_before_action = available[-1] if available else None
            if self._latest_state.provenance.observation_receive_sec > action_stamp:
                self._skipped_action_no_state += 1
                return
            if (
                latest_before_action is None
                or self._latest_state.provenance.official_trajectory
                != latest_before_action
            ):
                self._skipped_action_stale_trajectory += 1
                return
            self._pending_transition = {
                "state": self._latest_state,
                "latest_official_trajectory": latest_before_action,
                "requested": (action_stamp, requested_value),
                "filtered": (action_stamp, filtered_value),
                "applied": None,
            }
            self._last_state_key_used = state_key

    def _applied(self, message):
        value = self._finite_speed(message)
        if value is None:
            return
        now = self._now()
        with self._lock:
            if self._finalized:
                return
            self._latest_applied = (now, value)
            if (
                self._pending_transition is not None
                and self._pending_transition["filtered"] is not None
                and self._pending_transition["applied"] is None
                and now >= self._pending_transition["filtered"][0]
            ):
                expected = self._pending_transition["filtered"][1]
                if abs(value - expected) <= self._applied_pair_tolerance:
                    self._pending_transition["applied"] = (now, value)
                else:
                    self._ignored_unmatched_applied += 1

    def _policy_state(self, message, receive_sec):
        if not message.valid:
            return None
        positions = np.asarray(
            [[point.x, point.y, point.z] for point in message.future_positions_body],
            dtype=np.float32,
        )
        velocity = message.actual_velocity_body
        tracking = message.tracking_error_body
        trajectory = self._trajectory_identity_from_observation(message)
        return PolicyStateV1(
            lidar_surrogate=np.asarray(message.lidar_surrogate, dtype=np.float32),
            future_positions_body=positions,
            actual_velocity_body=np.asarray(
                [velocity.x, velocity.y, velocity.z], dtype=np.float32
            ),
            tracking_error_body=np.asarray(
                [tracking.x, tracking.y, tracking.z], dtype=np.float32
            ),
            previous_applied_v_max=float(message.previous_v_max),
            provenance=PolicyStateProvenance(
                observation_stamp_sec=message.header.stamp.to_sec(),
                observation_receive_sec=receive_sec,
                body_frame=message.header.frame_id.lstrip("/"),
                observation_version=message.version,
                official_trajectory=trajectory,
            ),
        )

    def _maybe_write_transition(self, next_state):
        pending = self._pending_transition
        if pending is None or pending["filtered"] is None or pending["applied"] is None:
            return
        if (
            next_state.provenance.observation_stamp_sec
            <= pending["state"].provenance.observation_stamp_sec
            or next_state.provenance.observation_receive_sec < pending["applied"][0]
        ):
            return
        requested_stamp, requested_value = pending["requested"]
        filtered_stamp, filtered_value = pending["filtered"]
        applied_stamp, applied_value = pending["applied"]
        dangerous = self._dangerous_terminal()
        mission_terminal = self._mission_done
        reasons = []
        if self._collision_terminal:
            reasons.append("collision_proxy")
        if self._emergency_terminal:
            reasons.append("ego_emergency_stop")
        if self._tracking_mirror.triggered:
            reasons.append("tracking_safety_gate")
        if mission_terminal and not reasons:
            reasons.append("mission_terminal")
        transition = SacTransitionV1(
            state_t=pending["state"],
            action_t=AppliedSpeedAction(
                requested_v_max=requested_value,
                requested_stamp_sec=requested_stamp,
                filtered_v_max=filtered_value,
                filtered_stamp_sec=filtered_stamp,
                applied_v_max=applied_value,
                applied_stamp_sec=applied_stamp,
                latest_official_trajectory=pending["latest_official_trajectory"],
            ),
            state_t_plus_1=next_state,
            reward=None,
            reward_defined=False,
            terminated=bool(dangerous or mission_terminal),
            truncated=False,
            terminal_reason=";".join(reasons),
        )
        self._transition_stream.write(json.dumps(transition.to_record()) + "\n")
        self._transition_stream.flush()
        self._transition_candidates += 1
        if transition.terminated or transition.truncated:
            self._transition_terminal_written = True
        self._pending_transition = None

    @staticmethod
    def _pair_value(sample):
        return "" if sample is None else sample[1]

    @staticmethod
    def _pair_stamp(sample):
        return "" if sample is None else sample[0]

    def _observation(self, message):
        receive_sec = self._now()
        with self._lock:
            if self._finalized:
                return
            self._observation_messages += 1
            state = None
            state_error = ""
            try:
                state = self._policy_state(message, receive_sec)
            except (TypeError, ValueError) as error:
                state_error = "contract_invalid:{}".format(error)
            if state is not None:
                self._latest_state = state
                self._valid_observations += 1
                self._maybe_write_transition(state)

            reasons = list(message.diagnostics)
            if state_error:
                reasons.append(state_error)
            row = {field: "" for field in SAMPLE_FIELDS}
            stamp = message.header.stamp.to_sec()
            row.update(
                {
                    "observation_stamp_sec": stamp,
                    "observation_receive_sec": receive_sec,
                    "observation_latency_sec": max(0.0, receive_sec - stamp),
                    "observation_version": message.version,
                    "observation_valid": int(state is not None),
                    "observation_invalid_reasons": ";".join(reasons),
                    "observation_frame": message.header.frame_id,
                    "mission_state": self._mission_state,
                    "planner_state": self._planner_state,
                    "planner_failure_active": int(self._planner_failure_active),
                    "planner_failure_reason": self._planner_failure_reason,
                    "planner_failure_episode_id": (
                        ""
                        if self._planner_episodes.active is None
                        else self._planner_episodes.active.episode_id
                    ),
                    "planner_consecutive_failures": self._planner_consecutive_failures,
                    "collision_safety_terminal": int(self._collision_terminal),
                    "emergency_safety_terminal": int(self._emergency_terminal),
                    "tracking_safety_terminal": int(self._tracking_mirror.triggered),
                    "dangerous_terminal": int(self._dangerous_terminal()),
                    "mission_success": int(self._mission_success),
                    "mission_failure": int(self._mission_failure),
                    "mission_done": int(self._mission_done),
                    "latest_requested_v_max_mps": self._pair_value(self._latest_requested),
                    "latest_requested_stamp_sec": self._pair_stamp(self._latest_requested),
                    "latest_filtered_v_max_mps": self._pair_value(self._latest_filtered),
                    "latest_filtered_stamp_sec": self._pair_stamp(self._latest_filtered),
                    "latest_action_receive_sec": (
                        "" if self._latest_action_receive_sec is None else self._latest_action_receive_sec
                    ),
                    "latest_applied_v_max_mps": self._pair_value(self._latest_applied),
                    "latest_applied_receive_sec": self._pair_stamp(self._latest_applied),
                    "raw_filtered_nearest_point_distance_m": self._pair_value(
                        self._latest_raw_clearance
                    ),
                    "raw_filtered_clearance_stamp_sec": self._pair_stamp(
                        self._latest_raw_clearance
                    ),
                    "inflated_occupied_center_distance_m": self._pair_value(
                        self._latest_inflated_clearance
                    ),
                    "inflated_clearance_stamp_sec": self._pair_stamp(
                        self._latest_inflated_clearance
                    ),
                }
            )
            if state is not None:
                clutter = lidar_clutter_metrics(
                    message.lidar_surrogate, message.lidar_semantic
                )
                velocity = state.actual_velocity_body
                tracking = state.tracking_error_body
                row.update(clutter)
                row.update(
                    {
                        "trajectory_id": state.provenance.official_trajectory.trajectory_id,
                        "trajectory_start_time_sec": state.provenance.official_trajectory.start_time_sec,
                        "trajectory_source_frame": state.provenance.official_trajectory.source_frame,
                        "actual_speed_mps": float(np.linalg.norm(velocity)),
                        "actual_velocity_x_body_mps": float(velocity[0]),
                        "actual_velocity_y_body_mps": float(velocity[1]),
                        "actual_velocity_z_body_mps": float(velocity[2]),
                        "tracking_error_x_body_m": float(tracking[0]),
                        "tracking_error_y_body_m": float(tracking[1]),
                        "tracking_error_z_body_m": float(tracking[2]),
                        "tracking_error_norm_m": float(np.linalg.norm(tracking)),
                        "previous_applied_v_max_mps": state.previous_applied_v_max,
                    }
                )
            self._sample_writer.writerow(row)
            self._sample_stream.flush()
            diagnostic_record = {
                "observation_stamp_sec": stamp,
                "observation_receive_sec": receive_sec,
                "version": message.version,
                "valid": state is not None,
                "diagnostics": reasons,
                "lidar_valid_mask": list(message.lidar_valid_mask),
                "lidar_unknown_mask": list(message.lidar_unknown_mask),
                "lidar_semantic": list(message.lidar_semantic),
                "mission_state": self._mission_state,
                "planner_state": self._planner_state,
                "planner_failure_reason": self._planner_failure_reason,
                "raw_filtered_nearest_point_distance_m": self._pair_value(
                    self._latest_raw_clearance
                ),
                "inflated_occupied_center_distance_m": self._pair_value(
                    self._latest_inflated_clearance
                ),
            }
            self._diagnostics_stream.write(json.dumps(diagnostic_record) + "\n")
            self._diagnostics_stream.flush()

    def _odom(self, message):
        position = message.pose.pose.position
        values = np.asarray([position.x, position.y, position.z], dtype=float)
        if not np.all(np.isfinite(values)):
            return
        with self._lock:
            if not self._finalized:
                self._latest_position_world = values
                self._latest_odom_frame = message.header.frame_id.lstrip("/")

    def _cloud_clearance(self, message, representation):
        stamp = message.header.stamp.to_sec()
        with self._lock:
            if (
                self._finalized
                or self._latest_position_world is None
                or message.header.frame_id.lstrip("/") != self._latest_odom_frame
                or stamp - self._last_clearance_stamp[representation]
                < 1.0 / self._clearance_rate_hz
            ):
                return
            center = self._latest_position_world.copy()
            self._last_clearance_stamp[representation] = stamp
        minimum = math.inf
        try:
            for point in point_cloud2.read_points(
                message, field_names=("x", "y", "z"), skip_nans=True
            ):
                distance = float(np.linalg.norm(np.asarray(point[:3]) - center))
                minimum = min(minimum, distance)
        except (KeyError, TypeError, ValueError) as error:
            rospy.logwarn_throttle(
                1.0, "calibration clearance decode failed: %s", error
            )
            return
        if not math.isfinite(minimum):
            return
        with self._lock:
            if representation == "raw":
                self._latest_raw_clearance = (stamp, minimum)
            else:
                self._latest_inflated_clearance = (stamp, minimum)

    def _planner(self, message):
        now = self._now()
        stamp = message.status_timestamp.to_sec()
        if stamp <= 0.0:
            stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = now
        failed = bool(
            message.consecutive_plan_failures > 0
            or message.emergency_stop_active
            or (
                not message.last_plan_success
                and message.failure_reason != PlannerStatus.NONE
            )
        )
        with self._lock:
            if self._finalized:
                return
            self._planner_state = message.planner_state
            self._planner_failure_active = failed
            self._planner_failure_reason = message.failure_reason
            self._planner_consecutive_failures = int(
                message.consecutive_plan_failures
            )
            self._collision_terminal = self._collision_terminal or bool(
                message.current_position_in_collision
            )
            self._emergency_terminal = self._emergency_terminal or bool(
                message.emergency_stop_active
            )
            self._planner_episodes.update(
                stamp,
                failed,
                message.failure_reason,
                int(message.consecutive_plan_failures),
            )

    def _state(self, message):
        with self._lock:
            self._mission_state = message.data

    def _success(self, message):
        with self._lock:
            self._mission_success = bool(message.data)

    def _failure(self, message):
        with self._lock:
            self._mission_failure = bool(message.data)

    def _done(self, message):
        with self._lock:
            self._mission_done = bool(message.data)
            if self._mission_done and not self._shutdown_scheduled:
                self._shutdown_scheduled = True
                rospy.Timer(
                    rospy.Duration(1.0), self._terminal_shutdown, oneshot=True
                )

    def _terminal_shutdown(self, _event):
        self.finalize()
        rospy.signal_shutdown("mission terminal calibration record finalized")

    def _bridge(self, message):
        with self._lock:
            self._bridge_state = message.data

    def _tracking_error(self, message):
        value = float(message.data)
        if not math.isfinite(value):
            return
        with self._lock:
            if not self._finalized:
                self._tracking_mirror.update(
                    self._now(), value, self._bridge_state
                )

    def _dangerous_terminal(self):
        return bool(
            self._collision_terminal
            or self._emergency_terminal
            or self._tracking_mirror.triggered
        )

    def finalize(self):
        with self._lock:
            if self._finalized:
                return
            self._finalized = True
            self._ended_sec = self._now()
            if self._pending_transition is not None:
                self._dropped_incomplete_transitions += 1
                self._pending_transition = None
            self._planner_episodes.close_unrecovered(
                max(self._ended_sec, self._started_sec)
            )
            episode_path = self._output_dir / "planner_failure_episodes.csv"
            with episode_path.open("w", newline="", encoding="utf-8") as stream:
                fields = (
                    "episode_id",
                    "start_stamp_sec",
                    "end_stamp_sec",
                    "duration_sec",
                    "recovered",
                    "reasons",
                    "maximum_consecutive_failures",
                )
                writer = csv.DictWriter(stream, fieldnames=fields)
                writer.writeheader()
                for episode in self._planner_episodes.completed:
                    writer.writerow(
                        {
                            "episode_id": episode.episode_id,
                            "start_stamp_sec": episode.start_stamp_sec,
                            "end_stamp_sec": episode.end_stamp_sec,
                            "duration_sec": (
                                None
                                if episode.end_stamp_sec is None
                                else max(
                                    0.0,
                                    episode.end_stamp_sec
                                    - episode.start_stamp_sec,
                                )
                            ),
                            "recovered": int(episode.recovered),
                            "reasons": ";".join(episode.reasons),
                            "maximum_consecutive_failures": episode.maximum_consecutive_failures,
                        }
                    )
            result = (
                "success"
                if self._mission_success
                else "failure"
                if self._mission_failure
                else "done_without_result"
                if self._mission_done
                else "incomplete"
            )
            summary = {
                "schema_version": "learning_speed_calibration_v1.0",
                "run_id": self._run_id,
                "started_sec": self._started_sec,
                "ended_sec": self._ended_sec,
                "reward_defined": False,
                "training_started": False,
                "observation_c": {
                    "messages": self._observation_messages,
                    "valid": self._valid_observations,
                    "valid_ratio": (
                        float(self._valid_observations)
                        / float(self._observation_messages)
                        if self._observation_messages
                        else None
                    ),
                },
                "transitions": {
                    "candidates": self._transition_candidates,
                    "training_ready": 0,
                    "skipped_action_no_new_state": self._skipped_action_no_state,
                    "skipped_action_latest_trajectory_mismatch": self._skipped_action_stale_trajectory,
                    "skipped_action_pending": self._skipped_action_pending,
                    "dropped_incomplete": self._dropped_incomplete_transitions,
                    "ignored_unmatched_applied_ack": self._ignored_unmatched_applied,
                },
                "safety": {
                    "collision_proxy_terminal": self._collision_terminal,
                    "collision_representation": "PlannerStatus.current_position_in_collision against EGO inflated occupancy; not physical Gazebo contact",
                    "emergency_terminal": self._emergency_terminal,
                    "tracking_safety_terminal": self._tracking_mirror.triggered,
                    "tracking_safety_source": "read-only mirror of bridge tracking_error > configured 1.0 m for configured 1.0 s while TRACK_EGO",
                    "dangerous_terminal": self._dangerous_terminal(),
                },
                "planner": {
                    "failure_episodes": len(self._planner_episodes.completed),
                    "recovered_episodes": sum(
                        int(episode.recovered)
                        for episode in self._planner_episodes.completed
                    ),
                },
                "mission": {
                    "state": self._mission_state,
                    "success": self._mission_success,
                    "failure": self._mission_failure,
                    "done": self._mission_done,
                    "result": result,
                },
                "episode": {
                    "terminated": bool(self._mission_done or self._dangerous_terminal()),
                    "truncated": bool(
                        not self._mission_done and not self._dangerous_terminal()
                    ),
                },
                "clutter_semantics": {
                    "nearest_obstacle_distance_m": "minimum known-obstacle distance encoded by the valid Observation C lidar surrogate",
                    "known_obstacle_bin_fraction": "known-obstacle angular bins divided by 3200; diagnostic density proxy, not clearance or physical volume",
                    "known_obstacle_bin_count": "occupied angular-bin clutter proxy, not object-instance count",
                },
                "clearance_semantics": {
                    "raw_filtered_nearest_point_distance_m": "UAV center to nearest point in stage3/cloud_registered_filtered; no inflation or vehicle-radius subtraction",
                    "inflated_occupied_center_distance_m": "UAV center to nearest occupied voxel center in EGO occupancy_inflate; already inflated and not physical surface clearance",
                    "sampling_rate_hz": self._clearance_rate_hz,
                },
            }
            with (self._output_dir / "run_summary.json").open(
                "w", encoding="utf-8"
            ) as stream:
                json.dump(summary, stream, indent=2, sort_keys=True)
                stream.write("\n")
            self._sample_stream.close()
            self._transition_stream.close()
            self._diagnostics_stream.close()


def main():
    rospy.init_node("learning_speed_calibration_recorder")
    CalibrationDataRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
