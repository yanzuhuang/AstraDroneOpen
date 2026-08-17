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
from collections import Counter, defaultdict

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
    causal_observation_receipt_time,
    lidar_clutter_metrics,
    validate_artifact_root,
)
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float64, String
from traj_utils.msg import Bspline


SAMPLE_FIELDS = (
    "environment",
    "training_active",
    "observation_stamp_sec",
    "observation_source_stamp_sec",
    "lidar_source_stamp_sec",
    "lookup_target_stamp_sec",
    "recorder_callback_ros_time_sec",
    "recorder_clock_lag_sec",
    "observation_receive_sec",
    "observation_latency_sec",
    "observation_version",
    "observation_valid",
    "observation_invalid_reasons",
    "observation_frame",
    "kinematic_lookup_result",
    "kinematic_lookup_failure_reason",
    "state_before_missing",
    "state_before_stamp_sec",
    "state_after_missing",
    "state_after_stamp_sec",
    "dt_before_sec",
    "dt_after_sec",
    "bracket_span_sec",
    "nearest_state_dt_sec",
    "state_buffer_oldest_stamp_sec",
    "state_buffer_newest_stamp_sec",
    "state_buffer_size",
    "state_buffer_coverage_sec",
    "latest_state_age_at_lookup_sec",
    "fast_lio_source_rate_hz",
    "fast_lio_state_insert_rate_hz",
    "state_insert_count",
    "out_of_order_state_count",
    "duplicate_state_stamp_count",
    "state_buffer_eviction_count",
    "source_to_receipt_latency_sec",
    "trajectory_lookup_result",
    "trajectory_history_size",
    "selected_trajectory_id",
    "selected_trajectory_start_stamp_sec",
    "selected_trajectory_end_stamp_sec",
    "latest_trajectory_id_at_lookup",
    "latest_trajectory_start_stamp_sec",
    "lidar_invalid_reason",
    "lidar_input_points",
    "lidar_finite_points",
    "lidar_in_range_points",
    "lidar_history_frames",
    "lidar_source_rate_hz",
    "lidar_build_duration_ms",
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


TRAINING_ACTIVE_STATES = frozenset((
    "STAGING_POINT",
    "ENTRY_GATE_TRANSIT",
    "NAVIGATING",
    "RELOCATING",
    "RECOVERING",
    "LAYER_TRANSITION",
    "GO_TO_EXIT_GATE",
    "NORMAL_RETURN",
    "RETURN_EGRESS",
))


class CalibrationDataRecorder:
    def __init__(self):
        self._lock = threading.RLock()
        self._run_id = str(rospy.get_param("~run_id", "manual")).strip()
        self._environment = str(rospy.get_param("~environment", "")).strip()
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
        self._mission_started = False
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
        self._training_active_messages = 0
        self._training_active_valid = 0
        self._invalid_reason_counts = Counter()
        self._invalid_reason_by_phase = defaultdict(Counter)
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

    @staticmethod
    def _message_time(message, field, missing_field=None):
        if missing_field is not None and getattr(message, missing_field):
            return ""
        value = getattr(message, field).to_sec()
        return value if value > 0.0 else ""

    def _observation(self, message):
        callback_ros_time = self._now()
        source_stamp = message.header.stamp.to_sec()
        producer_receive = message.lookup_receipt_time.to_sec()
        # /clock callbacks are delivered independently to each process. The
        # collector can therefore observe a slightly older ROS time even though
        # this message was causally received after Observation C produced it.
        receive_sec = causal_observation_receipt_time(
            callback_ros_time, producer_receive, source_stamp
        )
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
            stamp = source_stamp
            training_active = self._mission_state in TRAINING_ACTIVE_STATES
            if training_active:
                self._training_active_messages += 1
                if state is not None:
                    self._training_active_valid += 1
            if state is None:
                reason_key = ";".join(reasons) or "unspecified_invalid"
                self._invalid_reason_counts[reason_key] += 1
                self._invalid_reason_by_phase[self._mission_state or "<empty>"][reason_key] += 1
            row.update(
                {
                    "environment": self._environment,
                    "training_active": int(training_active),
                    "observation_stamp_sec": stamp,
                    "observation_source_stamp_sec": stamp,
                    "lidar_source_stamp_sec": stamp,
                    "lookup_target_stamp_sec": stamp,
                    "recorder_callback_ros_time_sec": callback_ros_time,
                    "recorder_clock_lag_sec": receive_sec - callback_ros_time,
                    "observation_receive_sec": receive_sec,
                    "observation_latency_sec": max(0.0, receive_sec - stamp),
                    "observation_version": message.version,
                    "observation_valid": int(state is not None),
                    "observation_invalid_reasons": ";".join(reasons),
                    "observation_frame": message.header.frame_id,
                    "kinematic_lookup_result": message.kinematic_lookup_result,
                    "kinematic_lookup_failure_reason": message.kinematic_lookup_failure_reason,
                    "state_before_missing": int(message.state_before_missing),
                    "state_before_stamp_sec": self._message_time(
                        message, "state_before_stamp", "state_before_missing"
                    ),
                    "state_after_missing": int(message.state_after_missing),
                    "state_after_stamp_sec": self._message_time(
                        message, "state_after_stamp", "state_after_missing"
                    ),
                    "dt_before_sec": (
                        "" if message.state_before_missing else message.dt_before_sec
                    ),
                    "dt_after_sec": (
                        "" if message.state_after_missing else message.dt_after_sec
                    ),
                    "bracket_span_sec": (
                        "" if message.state_before_missing or message.state_after_missing
                        else message.bracket_span_sec
                    ),
                    "nearest_state_dt_sec": (
                        "" if message.state_before_missing and message.state_after_missing
                        else message.nearest_state_dt_sec
                    ),
                    "state_buffer_oldest_stamp_sec": self._message_time(
                        message, "state_buffer_oldest_stamp",
                        "state_buffer_oldest_missing",
                    ),
                    "state_buffer_newest_stamp_sec": self._message_time(
                        message, "state_buffer_newest_stamp",
                        "state_buffer_newest_missing",
                    ),
                    "state_buffer_size": message.state_buffer_size,
                    "state_buffer_coverage_sec": message.state_buffer_coverage_sec,
                    "latest_state_age_at_lookup_sec": (
                        "" if message.state_buffer_newest_missing
                        else message.latest_state_age_at_lookup_sec
                    ),
                    "fast_lio_source_rate_hz": message.fast_lio_source_rate_hz,
                    "fast_lio_state_insert_rate_hz": message.fast_lio_state_insert_rate_hz,
                    "state_insert_count": message.state_insert_count,
                    "out_of_order_state_count": message.out_of_order_state_count,
                    "duplicate_state_stamp_count": message.duplicate_state_stamp_count,
                    "state_buffer_eviction_count": message.state_buffer_eviction_count,
                    "source_to_receipt_latency_sec": message.source_to_receipt_latency_sec,
                    "trajectory_lookup_result": message.trajectory_lookup_result,
                    "trajectory_history_size": message.trajectory_history_size,
                    "selected_trajectory_id": (
                        "" if message.selected_trajectory_missing
                        else message.selected_trajectory_id
                    ),
                    "selected_trajectory_start_stamp_sec": self._message_time(
                        message, "selected_trajectory_start_stamp",
                        "selected_trajectory_missing",
                    ),
                    "selected_trajectory_end_stamp_sec": self._message_time(
                        message, "selected_trajectory_end_stamp",
                        "selected_trajectory_missing",
                    ),
                    "latest_trajectory_id_at_lookup": (
                        "" if message.latest_trajectory_missing
                        else message.latest_trajectory_id_at_lookup
                    ),
                    "latest_trajectory_start_stamp_sec": self._message_time(
                        message, "latest_trajectory_start_stamp",
                        "latest_trajectory_missing",
                    ),
                    "lidar_invalid_reason": message.lidar_invalid_reason,
                    "lidar_input_points": message.lidar_input_points,
                    "lidar_finite_points": message.lidar_finite_points,
                    "lidar_in_range_points": message.lidar_in_range_points,
                    "lidar_history_frames": message.lidar_history_frames,
                    "lidar_source_rate_hz": message.lidar_source_rate_hz,
                    "lidar_build_duration_ms": message.lidar_build_duration_ms,
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
                "environment": self._environment,
                "training_active": training_active,
                "observation_stamp_sec": stamp,
                "observation_source_stamp_sec": stamp,
                "lidar_source_stamp_sec": stamp,
                "lookup_target_stamp_sec": stamp,
                "recorder_callback_ros_time_sec": callback_ros_time,
                "recorder_clock_lag_sec": receive_sec - callback_ros_time,
                "observation_receive_sec": receive_sec,
                "version": message.version,
                "valid": state is not None,
                "diagnostics": reasons,
                "kinematic_lookup": {
                    "result": message.kinematic_lookup_result,
                    "failure_reason": message.kinematic_lookup_failure_reason,
                    "before_missing": bool(message.state_before_missing),
                    "before_stamp_sec": row["state_before_stamp_sec"],
                    "after_missing": bool(message.state_after_missing),
                    "after_stamp_sec": row["state_after_stamp_sec"],
                    "dt_before_sec": row["dt_before_sec"],
                    "dt_after_sec": row["dt_after_sec"],
                    "bracket_span_sec": row["bracket_span_sec"],
                    "nearest_state_dt_sec": row["nearest_state_dt_sec"],
                    "buffer_oldest_stamp_sec": row["state_buffer_oldest_stamp_sec"],
                    "buffer_newest_stamp_sec": row["state_buffer_newest_stamp_sec"],
                    "buffer_size": message.state_buffer_size,
                    "buffer_coverage_sec": message.state_buffer_coverage_sec,
                    "latest_state_age_at_lookup_sec": row["latest_state_age_at_lookup_sec"],
                    "fast_lio_source_rate_hz": message.fast_lio_source_rate_hz,
                    "state_insert_rate_hz": message.fast_lio_state_insert_rate_hz,
                    "insert_count": message.state_insert_count,
                    "out_of_order_state_count": message.out_of_order_state_count,
                    "duplicate_state_stamp_count": message.duplicate_state_stamp_count,
                    "eviction_count": message.state_buffer_eviction_count,
                    "receipt_stamp_sec": self._message_time(
                        message, "lookup_receipt_time"
                    ),
                    "source_to_receipt_latency_sec": message.source_to_receipt_latency_sec,
                },
                "trajectory_lookup": {
                    "result": message.trajectory_lookup_result,
                    "history_size": message.trajectory_history_size,
                    "selected_trajectory_id": row["selected_trajectory_id"],
                    "selected_start_stamp_sec": row["selected_trajectory_start_stamp_sec"],
                    "selected_end_stamp_sec": row["selected_trajectory_end_stamp_sec"],
                    "latest_trajectory_id_at_lookup": row["latest_trajectory_id_at_lookup"],
                    "latest_start_stamp_sec": row["latest_trajectory_start_stamp_sec"],
                },
                "lidar_source": {
                    "invalid_reason": message.lidar_invalid_reason,
                    "input_points": message.lidar_input_points,
                    "finite_points": message.lidar_finite_points,
                    "in_range_points": message.lidar_in_range_points,
                    "history_frames": message.lidar_history_frames,
                    "source_rate_hz": message.lidar_source_rate_hz,
                    "build_duration_ms": message.lidar_build_duration_ms,
                },
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
            # Ground/startup occupancy and planner warm-up are not part of a
            # flight episode.  Keep their latest diagnostics visible, but do
            # not turn them into calibration terminals or failure episodes
            # until the mission has actually left WAIT_INPUTS.
            if not self._mission_started or self._mission_done:
                return
            # The EGO map legitimately contains the grounded vehicle before
            # control hand-off.  The bridge's TRACK_EGO state is the existing
            # boundary at which this occupancy flag becomes a flight-safety
            # collision proxy; the collector remains read-only.
            self._collision_terminal = self._collision_terminal or bool(
                message.current_position_in_collision
                and self._bridge_state == "TRACK_EGO"
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
            if message.data not in ("", "WAIT_INPUTS"):
                self._mission_started = True

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
                    "training_active_messages": self._training_active_messages,
                    "training_active_valid": self._training_active_valid,
                    "training_active_valid_ratio": (
                        float(self._training_active_valid)
                        / float(self._training_active_messages)
                        if self._training_active_messages else None
                    ),
                    "training_active_states": sorted(TRAINING_ACTIVE_STATES),
                    "invalid_reasons": dict(self._invalid_reason_counts),
                    "invalid_reasons_by_mission_phase": {
                        phase: dict(counts)
                        for phase, counts in sorted(
                            self._invalid_reason_by_phase.items()
                        )
                    },
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
