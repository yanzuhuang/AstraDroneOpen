#!/usr/bin/env python3
"""Read-only Scheme C feature fusion; never publishes policy or control output."""

import math
import threading
import time
from collections import deque

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from std_srvs.srv import Trigger, TriggerResponse
from traj_utils.msg import Bspline

from learning_speed_rl.msg import (
    LidarSurrogateStamped,
    ObservationC as ObservationCMessage,
)
from learning_speed_rl.observation.latest_sample import LatestSampleMailbox
from learning_speed_rl.observation.scheme_c import (
    EgoBsplineTrajectory,
    ActiveTrajectoryStore,
    KinematicStateBuffer,
    LidarSurrogateFeature,
    OBSERVATION_C_VERSION,
    ObservationCBuilder,
    TrajectorySampler,
    TrajectorySamplingConfig,
    TimestampedScalarBuffer,
)
from learning_speed_rl.observation.v2 import Pose3D


def _key_value(key, value):
    return KeyValue(key=str(key), value=str(value))


class ObservationCNode:
    def __init__(self):
        self._lock = threading.RLock()
        root = "~observation_c/"
        param = lambda name, default: rospy.get_param(root + name, default)
        topic = lambda name, default: str(param("topics/" + name, default))

        if not bool(param("enabled", True)):
            raise rospy.ROSInitException("Observation C is explicitly disabled")
        self._world_frame = str(param("frames/world", "camera_init")).lstrip("/")
        self._body_frame = str(param("frames/body", "body")).lstrip("/")
        self._expected_bins = int(param("expected_lidar_bins", 3200))
        self._minimum_v_max = float(param("system_state/minimum_v_max", 0.01))
        self._maximum_v_max = float(param("system_state/maximum_v_max", 10.0))
        self._state_source_type = str(
            param("system_state/source_type", "fast_lio_state_estimate")
        )
        self._state_contract_version = str(
            param(
                "system_state/source_contract_version",
                "astradrone_planning_odometry_v1.0",
            )
        )
        self._state_velocity_source = str(
            param(
                "system_state/velocity_source",
                "timestamped_state_position_difference_world",
            )
        )
        self._state_lookup_policy = str(
            param("timing/state_lookup_policy", "interpolate")
        )
        self._require_lidar_state_provenance = bool(
            param("timing/require_lidar_state_provenance", False)
        )
        sampling = TrajectorySamplingConfig(
            sample_count=int(param("trajectory_sample_count", 20)),
            sample_spacing=float(param("trajectory_sample_spacing", 0.25)),
            max_distance=float(param("trajectory_max_distance", 5.0)),
            sampling_mode=str(param("trajectory_sampling_mode", "distance")),
            arc_length_resolution_m=float(param("arc_length_resolution_m", 0.02)),
            arc_length_max_depth=int(param("arc_length_max_depth", 16)),
        )
        self._builder = ObservationCBuilder(
            TrajectorySampler(sampling), self._world_frame, self._body_frame,
            self._state_source_type, self._state_contract_version,
            self._state_velocity_source,
        )
        self._state_buffer = KinematicStateBuffer(
            capacity=int(param("timing/state_buffer_capacity", 400)),
            maximum_interpolation_gap_sec=float(
                param("timing/maximum_state_interpolation_gap_sec", 0.05)
            ),
            velocity_filter_alpha=float(
                param("system_state/velocity_filter_alpha", 0.30)
            ),
            maximum_velocity_dt_sec=float(
                param("system_state/maximum_velocity_dt_sec", 0.50)
            ),
            lookup_policy=self._state_lookup_policy,
        )
        self._maximum_input_age = float(param("timing/maximum_input_age_sec", 0.25))
        self._diagnostics_rate = float(param("timing/diagnostics_rate_hz", 2.0))
        if (
            not self._world_frame
            or not self._body_frame
            or self._expected_bins <= 0
            or not 0.0 < self._minimum_v_max <= self._maximum_v_max
            or self._maximum_input_age <= 0.0
            or self._diagnostics_rate <= 0.0
            or self._state_lookup_policy not in (
                "interpolate", "causal_at_or_before"
            )
        ):
            raise rospy.ROSInitException("invalid Observation C configuration")

        self._trajectory_store = ActiveTrajectoryStore()
        self._v_max_buffer = TimestampedScalarBuffer(
            int(param("system_state/v_max_buffer_capacity", 100))
        )
        self._lidar_valid = False
        # V2 is a state snapshot stream.  Processing obsolete queued packets
        # increases latency without adding policy information, so both the
        # subscriber mailbox and the pose-wait mailbox retain only the latest
        # sample.
        self._pending_lidar = deque(maxlen=1)
        self._lidar_condition = threading.Condition(self._lock)
        self._lidar_mailbox = LatestSampleMailbox()
        self._lidar_input_count = 0
        self._lidar_processed_count = 0
        self._lidar_coalesced_drop_count = 0
        self._lidar_pose_wait_replace_count = 0
        self._lidar_worker_busy = False
        self._lidar_worker_started_wall_sec = None
        self._last_lidar_queue_lag_wall_sec = 0.0
        self._max_lidar_queue_lag_wall_sec = 0.0
        self._last_lidar_source_to_callback_sec = 0.0
        self._last_lidar_worker_total_wall_ms = 0.0
        self._max_lidar_worker_total_wall_ms = 0.0
        self._last_lookup_wall_ms = 0.0
        self._last_builder_wall_ms = 0.0
        self._last_message_build_wall_ms = 0.0
        self._last_publish_call_wall_ms = 0.0
        self._last_observation = None
        self._last_observation_receive_sec = None
        self._last_failure = "waiting_for_data"
        self._pose_lookup_mode = ""
        self._accepted_replans = 0
        self._rejected_trajectories = 0
        self._time_reset_count = 0
        self._temporal_generation = 0
        self._reset_barrier_stamp = 0.0
        self._pre_barrier_state_reject_count = 0
        self._pre_barrier_lidar_reject_count = 0
        self._pre_barrier_trajectory_reject_count = 0
        self._pre_barrier_v_max_reject_count = 0
        self._generation_mismatch_count = 0
        self._future_state_use_count = 0
        self._future_trajectory_use_count = 0

        self._observation_topic = topic("observation", "learning_speed/observation_c")
        self._valid_topic = topic("valid", "learning_speed/observation_c/valid")
        self._diagnostics_topic = topic(
            "diagnostics", "learning_speed/observation_c/diagnostics"
        )
        self._observation_pub = rospy.Publisher(
            self._observation_topic, ObservationCMessage, queue_size=1
        )
        self._valid_pub = rospy.Publisher(
            self._valid_topic, Bool, queue_size=1, latch=True
        )
        self._diagnostic_pub = rospy.Publisher(
            self._diagnostics_topic, DiagnosticArray, queue_size=1
        )

        self._odom_sub = rospy.Subscriber(
            topic("odom", "Odometry"), Odometry, self._odom_callback, queue_size=100
        )
        self._trajectory_sub = rospy.Subscriber(
            topic("trajectory", "planning/bspline"),
            Bspline,
            self._trajectory_callback,
            queue_size=10,
        )
        self._lidar_sub = rospy.Subscriber(
            topic("lidar_stamped", "learning_speed/observation_v2/stamped"),
            LidarSurrogateStamped,
            self._lidar_callback,
            queue_size=1,
        )
        self._lidar_valid_sub = rospy.Subscriber(
            topic("lidar_valid", "learning_speed/observation_v2/valid"),
            Bool,
            self._lidar_valid_callback,
            queue_size=2,
        )
        self._v_max_sub = rospy.Subscriber(
            topic("ego_applied_v_max", "learning_speed/applied_v_max"),
            Float64,
            self._v_max_callback,
            queue_size=2,
        )
        self._clear_service = rospy.Service(
            "~clear_temporal_history", Trigger, self._clear_temporal_history
        )
        self._diagnostic_timer = rospy.Timer(
            rospy.Duration(1.0 / self._diagnostics_rate), self._diagnostic_callback
        )
        self._lidar_worker = threading.Thread(
            target=self._lidar_worker_loop,
            name="observation_c_latest_v2_worker",
            daemon=True,
        )
        self._lidar_worker.start()
        self._valid_pub.publish(Bool(data=False))
        rospy.logwarn(
            "Observation C read-only fusion active: trajectory=%s lidar=%s; no policy/v_max/control publisher exists",
            rospy.resolve_name(topic("trajectory", "planning/bspline")),
            rospy.resolve_name(topic("lidar_stamped", "learning_speed/observation_v2/stamped")),
        )

    @staticmethod
    def _set_time(message, field, value):
        if value is not None and math.isfinite(value) and value > 0.0:
            setattr(message, field, rospy.Time.from_sec(value))

    def _populate_lookup_diagnostics(
        self, packet, kinematic=None, trajectory=None, lidar_message=None,
        receipt_sec=None,
    ):
        packet.temporal_generation = self._temporal_generation
        packet.reset_barrier_stamp = rospy.Time.from_sec(
            max(0.0, self._reset_barrier_stamp)
        )
        packet.state_before_missing = True
        packet.state_after_missing = True
        packet.state_buffer_oldest_missing = True
        packet.state_buffer_newest_missing = True
        packet.selected_trajectory_missing = True
        packet.latest_trajectory_missing = True
        packet.selected_trajectory_id = -1
        packet.latest_trajectory_id_at_lookup = -1
        packet.dt_before_sec = -1.0
        packet.dt_after_sec = -1.0
        packet.bracket_span_sec = -1.0
        packet.nearest_state_dt_sec = -1.0
        packet.latest_state_age_at_lookup_sec = -1.0
        packet.source_to_receipt_latency_sec = -1.0
        if receipt_sec is not None and math.isfinite(receipt_sec):
            self._set_time(packet, "lookup_receipt_time", receipt_sec)
            source_stamp = packet.header.stamp.to_sec()
            if source_stamp > 0.0:
                packet.source_to_receipt_latency_sec = receipt_sec - source_stamp
        if kinematic is not None:
            packet.kinematic_lookup_result = kinematic.result
            packet.kinematic_lookup_failure_reason = kinematic.failure_reason
            packet.state_before_missing = kinematic.before_stamp_sec is None
            packet.state_after_missing = kinematic.after_stamp_sec is None
            packet.state_buffer_oldest_missing = kinematic.oldest_stamp_sec is None
            packet.state_buffer_newest_missing = kinematic.newest_stamp_sec is None
            self._set_time(packet, "state_before_stamp", kinematic.before_stamp_sec)
            self._set_time(packet, "state_after_stamp", kinematic.after_stamp_sec)
            self._set_time(
                packet, "state_buffer_oldest_stamp", kinematic.oldest_stamp_sec
            )
            self._set_time(
                packet, "state_buffer_newest_stamp", kinematic.newest_stamp_sec
            )
            for field in (
                "dt_before_sec", "dt_after_sec", "bracket_span_sec",
                "nearest_state_dt_sec", "latest_state_age_at_lookup_sec",
            ):
                value = getattr(kinematic, field)
                if value is not None and math.isfinite(value):
                    setattr(packet, field, value)
            packet.state_buffer_size = kinematic.buffer_size
            packet.state_buffer_coverage_sec = kinematic.buffer_coverage_sec
            packet.state_source_type = self._state_source_type
            packet.state_contract_version = self._state_contract_version
            packet.state_velocity_source = self._state_velocity_source
            packet.state_source_rate_hz = kinematic.source_rate_hz
            packet.state_insert_rate_hz = kinematic.insert_rate_hz
            # Frozen v1 compatibility aliases; do not infer FAST-LIO source
            # from these names. Generic fields above are authoritative.
            packet.fast_lio_source_rate_hz = kinematic.source_rate_hz
            packet.fast_lio_state_insert_rate_hz = kinematic.insert_rate_hz
            packet.state_insert_count = kinematic.insert_count
            packet.out_of_order_state_count = kinematic.out_of_order_state_count
            packet.duplicate_state_stamp_count = (
                kinematic.duplicate_state_stamp_count
            )
            packet.state_buffer_eviction_count = kinematic.eviction_count
        if trajectory is not None:
            packet.trajectory_lookup_result = trajectory.result
            packet.trajectory_history_size = trajectory.history_size
            packet.selected_trajectory_missing = (
                trajectory.selected_trajectory_id is None
            )
            if trajectory.selected_trajectory_id is not None:
                packet.selected_trajectory_id = trajectory.selected_trajectory_id
                self._set_time(
                    packet, "selected_trajectory_start_stamp",
                    trajectory.selected_start_stamp_sec,
                )
                self._set_time(
                    packet, "selected_trajectory_end_stamp",
                    trajectory.selected_end_stamp_sec,
                )
            packet.latest_trajectory_missing = (
                trajectory.latest_trajectory_id is None
            )
            if trajectory.latest_trajectory_id is not None:
                packet.latest_trajectory_id_at_lookup = (
                    trajectory.latest_trajectory_id
                )
                self._set_time(
                    packet, "latest_trajectory_start_stamp",
                    trajectory.latest_start_stamp_sec,
                )
        if lidar_message is not None:
            packet.lidar_temporal_generation = lidar_message.temporal_generation
            packet.lidar_pose_source_stamp = lidar_message.pose_source_stamp
            packet.lidar_pose_used_future = lidar_message.pose_used_future
            packet.lidar_invalid_reason = (
                "" if lidar_message.valid
                else ";".join(lidar_message.diagnostics) or "unspecified"
            )
            packet.lidar_input_points = lidar_message.input_points
            packet.lidar_finite_points = lidar_message.finite_points
            packet.lidar_in_range_points = lidar_message.in_range_points
            packet.lidar_history_frames = lidar_message.history_frames
            packet.lidar_source_rate_hz = lidar_message.lidar_source_rate_hz
            packet.lidar_build_duration_ms = lidar_message.build_duration_ms

    def _set_invalid(
        self, reason, stamp_sec=None, publish_packet=True,
        kinematic=None, trajectory=None, lidar_message=None, receipt_sec=None,
    ):
        rospy.logwarn_throttle(1.0, "Observation C invalid: %s", reason)
        with self._lock:
            self._last_failure = str(reason)
            self._last_observation = None
        self._valid_pub.publish(Bool(data=False))
        if publish_packet:
            packet = ObservationCMessage()
            value = rospy.Time.now().to_sec() if stamp_sec is None else stamp_sec
            packet.header.stamp = rospy.Time.from_sec(max(0.0, value))
            packet.header.frame_id = self._body_frame
            packet.version = OBSERVATION_C_VERSION
            packet.valid = False
            packet.diagnostics = [str(reason)]
            self._populate_lookup_diagnostics(
                packet, kinematic, trajectory, lidar_message, receipt_sec
            )
            self._observation_pub.publish(packet)

    def _clear_for_time_reset(
        self, reason="ros_time_reset", barrier_stamp=0.0, new_generation=True
    ):
        self._state_buffer.clear()
        self._pending_lidar.clear()
        self._lidar_mailbox.clear()
        self._trajectory_store.clear()
        self._v_max_buffer.clear()
        self._last_observation = None
        self._last_observation_receive_sec = None
        self._time_reset_count += 1
        if new_generation:
            self._temporal_generation += 1
        self._reset_barrier_stamp = float(barrier_stamp)
        self._lidar_valid = False
        self._set_invalid(reason)

    def _clear_temporal_history(self, _request):
        barrier = rospy.Time.now().to_sec()
        if barrier <= 0.0:
            return TriggerResponse(
                success=False, message="ROS simulation time is not active"
            )
        with self._lock:
            self._clear_for_time_reset(
                "explicit_reset_barrier", barrier_stamp=barrier,
                new_generation=True,
            )
            generation = self._temporal_generation
        return TriggerResponse(
            success=True,
            message="generation={};barrier={:.9f}".format(generation, barrier),
        )

    def _odom_callback(self, message):
        receipt_sec = rospy.Time.now().to_sec()
        stamp = message.header.stamp.to_sec()
        world = message.header.frame_id.lstrip("/")
        body = message.child_frame_id.lstrip("/")
        if stamp <= 0.0:
            self._set_invalid("odom_stamp_invalid", publish_packet=False)
            return
        if world != self._world_frame or body != self._body_frame:
            self._set_invalid(
                "odom_frame_mismatch:{}->{},expected:{}->{}".format(
                    world, body, self._world_frame, self._body_frame
                ),
                stamp,
                publish_packet=False,
            )
            return
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        try:
            pose = Pose3D(
                stamp,
                np.asarray([position.x, position.y, position.z]),
                np.asarray([orientation.x, orientation.y, orientation.z, orientation.w]),
            )
        except ValueError as error:
            self._set_invalid("invalid_odometry:{}".format(error), stamp, False)
            return
        pending = None
        with self._lock:
            if stamp <= self._reset_barrier_stamp + 1.0e-9:
                self._pre_barrier_state_reject_count += 1
                return
            if self._state_buffer.add_pose(pose, receipt_sec):
                self._clear_for_time_reset(
                    "ros_time_reset", barrier_stamp=0.0,
                    new_generation=True,
                )
                return
            if (
                self._pending_lidar
                and self._pending_lidar[-1][0].header.stamp.to_sec()
                <= stamp + 1.0e-9
            ):
                pending = self._pending_lidar.pop()
                self._pending_lidar.clear()
                self._enqueue_lidar_locked(pending)
                self._lidar_condition.notify_all()

    def _trajectory_callback(self, message):
        try:
            points = np.asarray(
                [[point.x, point.y, point.z] for point in message.pos_pts],
                dtype=np.float64,
            )
            candidate = EgoBsplineTrajectory(
                degree=int(message.order),
                control_points=points,
                knots=np.asarray(message.knots, dtype=np.float64),
                start_time_sec=message.start_time.to_sec(),
                trajectory_id=int(message.traj_id),
                frame_id=message.frame_id.lstrip("/"),
            )
            if candidate.frame_id != self._world_frame:
                raise ValueError("trajectory frame mismatch")
        except (TypeError, ValueError) as error:
            with self._lock:
                self._rejected_trajectories += 1
            self._set_invalid("trajectory_invalid:{}".format(error), publish_packet=False)
            return
        with self._lock:
            if candidate.start_time_sec <= self._reset_barrier_stamp + 1.0e-9:
                self._pre_barrier_trajectory_reject_count += 1
                return
            current = self._trajectory_store.current
            if not self._trajectory_store.update(candidate):
                self._rejected_trajectories += 1
                return
            if current is not None:
                self._accepted_replans += 1

    def _lidar_valid_callback(self, message):
        with self._lock:
            self._lidar_valid = bool(message.data)

    def _v_max_callback(self, message):
        value = float(message.data)
        if not math.isfinite(value) or not self._minimum_v_max <= value <= self._maximum_v_max:
            self._set_invalid("previous_v_max_invalid", publish_packet=False)
            return
        with self._lock:
            receipt_sec = rospy.Time.now().to_sec()
            if receipt_sec <= self._reset_barrier_stamp + 1.0e-9:
                self._pre_barrier_v_max_reject_count += 1
                return
            self._v_max_buffer.add(receipt_sec, value)

    def _lidar_callback(self, message):
        callback_ros_sec = rospy.Time.now().to_sec()
        callback_wall_sec = time.monotonic()
        envelope = (message, callback_ros_sec, callback_wall_sec)
        with self._lidar_condition:
            self._lidar_input_count += 1
            self._enqueue_lidar_locked(envelope)
            self._lidar_condition.notify_all()

    def _enqueue_lidar_locked(self, envelope):
        if self._lidar_mailbox.push(envelope):
            self._lidar_coalesced_drop_count += 1

    def _lidar_worker_loop(self):
        while not rospy.is_shutdown():
            with self._lidar_condition:
                while (
                    not self._lidar_mailbox.pending
                    and not rospy.is_shutdown()
                ):
                    self._lidar_condition.wait(0.1)
                if rospy.is_shutdown():
                    return
                envelope = self._lidar_mailbox.pop()
                self._lidar_worker_busy = True
                self._lidar_worker_started_wall_sec = time.monotonic()
            message, callback_ros_sec, callback_wall_sec = envelope
            worker_start_wall = time.monotonic()
            queue_lag = max(0.0, worker_start_wall - callback_wall_sec)
            stamp = message.header.stamp.to_sec()
            with self._lock:
                self._last_lidar_queue_lag_wall_sec = queue_lag
                self._max_lidar_queue_lag_wall_sec = max(
                    self._max_lidar_queue_lag_wall_sec, queue_lag
                )
                self._last_lidar_source_to_callback_sec = max(
                    0.0, callback_ros_sec - stamp
                )
            try:
                self._process_lidar(
                    message,
                    allow_pending=True,
                    input_envelope=envelope,
                )
            finally:
                with self._lidar_condition:
                    worker_total_ms = (
                        time.monotonic() - worker_start_wall
                    ) * 1000.0
                    self._last_lidar_worker_total_wall_ms = worker_total_ms
                    self._max_lidar_worker_total_wall_ms = max(
                        self._max_lidar_worker_total_wall_ms,
                        worker_total_ms,
                    )
                    self._lidar_processed_count += 1
                    self._lidar_worker_busy = False
                    self._lidar_worker_started_wall_sec = None
                    self._lidar_condition.notify_all()

    def _process_lidar(self, message, allow_pending, input_envelope=None):
        fusion_start_wall = time.monotonic()
        receipt_sec = rospy.Time.now().to_sec()
        stamp = message.header.stamp.to_sec()
        frame = message.header.frame_id.lstrip("/")
        if stamp <= self._reset_barrier_stamp + 1.0e-9:
            with self._lock:
                self._pre_barrier_lidar_reject_count += 1
            self._set_invalid(
                "lidar_at_or_before_reset_barrier", stamp,
                lidar_message=message, receipt_sec=receipt_sec,
            )
            return
        if message.temporal_generation != self._temporal_generation:
            with self._lock:
                self._generation_mismatch_count += 1
            self._set_invalid(
                "lidar_temporal_generation_mismatch", stamp,
                lidar_message=message, receipt_sec=receipt_sec,
            )
            return
        if self._require_lidar_state_provenance and (
            message.state_source_type != self._state_source_type
            or message.state_contract_version != self._state_contract_version
        ):
            self._set_invalid(
                "lidar_state_source_provenance_mismatch", stamp,
                lidar_message=message, receipt_sec=receipt_sec,
            )
            return
        if message.pose_used_future:
            with self._lock:
                self._future_state_use_count += 1
            self._set_invalid(
                "lidar_used_future_pose", stamp,
                lidar_message=message, receipt_sec=receipt_sec,
            )
            return
        if stamp <= 0.0 or frame != self._body_frame:
            detail = ";".join(message.diagnostics) or "stamp_or_frame_invalid"
            reason = (
                "lidar_surrogate_invalid:{}".format(detail)
                if not message.valid else "lidar_stamp_or_frame_invalid"
            )
            self._set_invalid(
                reason, stamp, lidar_message=message, receipt_sec=receipt_sec
            )
            return
        lookup_start_wall = time.monotonic()
        with self._lock:
            state, lookup, kinematic_diagnostics = (
                self._state_buffer.lookup_with_diagnostics(stamp, receipt_sec)
            )
            trajectory, trajectory_diagnostics = (
                self._trajectory_store.lookup_with_diagnostics(stamp)
            )
            previous_v_max, v_max_lookup = self._v_max_buffer.lookup(stamp)
        lookup_wall_ms = (time.monotonic() - lookup_start_wall) * 1000.0
        if (
            self._state_lookup_policy == "causal_at_or_before"
            and kinematic_diagnostics.after_stamp_sec is not None
            and kinematic_diagnostics.after_stamp_sec > stamp + 1.0e-9
        ):
            with self._lock:
                self._future_state_use_count += 1
            self._set_invalid(
                "kinematic_lookup_used_future_state", stamp,
                kinematic=kinematic_diagnostics,
                trajectory=trajectory_diagnostics,
                lidar_message=message,
                receipt_sec=receipt_sec,
            )
            return
        if (
            trajectory is not None
            and trajectory.start_time_sec > stamp + 1.0e-9
        ):
            with self._lock:
                self._future_trajectory_use_count += 1
            self._set_invalid(
                "trajectory_future_leak", stamp,
                kinematic=kinematic_diagnostics,
                trajectory=trajectory_diagnostics,
                lidar_message=message,
                receipt_sec=receipt_sec,
            )
            return
        if not message.valid:
            detail = ";".join(message.diagnostics) or "unspecified"
            self._set_invalid(
                "lidar_surrogate_invalid:{}".format(detail), stamp,
                kinematic=kinematic_diagnostics,
                trajectory=trajectory_diagnostics,
                lidar_message=message,
                receipt_sec=receipt_sec,
            )
            return
        if state is None:
            if allow_pending and lookup == "observation_newer_than_kinematic_history":
                with self._lock:
                    envelope = input_envelope or (
                        message, receipt_sec, time.monotonic()
                    )
                    if self._pending_lidar:
                        self._lidar_pose_wait_replace_count += 1
                    self._pending_lidar.clear()
                    self._pending_lidar.append(envelope)
                self._set_invalid(
                    "waiting_for_timestamped_kinematic_state", stamp,
                    kinematic=kinematic_diagnostics,
                    trajectory=trajectory_diagnostics,
                    lidar_message=message,
                    receipt_sec=receipt_sec,
                )
                return
            self._set_invalid(
                lookup, stamp,
                kinematic=kinematic_diagnostics,
                trajectory=trajectory_diagnostics,
                lidar_message=message,
                receipt_sec=receipt_sec,
            )
            return
        if trajectory is None:
            self._set_invalid(
                "trajectory_unavailable", stamp,
                kinematic=kinematic_diagnostics,
                trajectory=trajectory_diagnostics,
                lidar_message=message,
                receipt_sec=receipt_sec,
            )
            return
        if previous_v_max is None:
            self._set_invalid(
                v_max_lookup, stamp,
                kinematic=kinematic_diagnostics,
                trajectory=trajectory_diagnostics,
                lidar_message=message,
                receipt_sec=receipt_sec,
            )
            return
        builder_start_wall = time.monotonic()
        try:
            semantic = (
                np.frombuffer(message.semantic, dtype=np.uint8)
                if isinstance(message.semantic, (bytes, bytearray))
                else np.asarray(message.semantic, dtype=np.uint8)
            )
            lidar = LidarSurrogateFeature(
                stamp_sec=stamp,
                frame_id=frame,
                surrogate=np.asarray(message.lidar_surrogate),
                valid_mask=np.asarray(message.lidar_valid_mask),
                unknown_mask=np.asarray(message.unknown_mask),
                semantic=semantic,
            )
            if lidar.surrogate.size != self._expected_bins:
                raise ValueError("lidar bin count mismatch")
            observation = self._builder.build(
                lidar, trajectory, state, previous_v_max
            )
        except ValueError as error:
            self._set_invalid(
                "fusion_invalid:{}".format(error), stamp,
                kinematic=kinematic_diagnostics,
                trajectory=trajectory_diagnostics,
                lidar_message=message,
                receipt_sec=receipt_sec,
            )
            return
        builder_wall_ms = (time.monotonic() - builder_start_wall) * 1000.0
        with self._lock:
            if (
                message.temporal_generation != self._temporal_generation
                or stamp <= self._reset_barrier_stamp + 1.0e-9
            ):
                self._generation_mismatch_count += 1
                return
        publish_phases = self._publish_observation(
            observation, kinematic_diagnostics, trajectory_diagnostics,
            message, receipt_sec,
            fusion_duration_wall_ms=(
                time.monotonic() - fusion_start_wall
            ) * 1000.0,
        )
        with self._lock:
            self._last_observation = observation
            self._last_observation_receive_sec = receipt_sec
            self._last_failure = ""
            self._pose_lookup_mode = lookup
            self._last_lookup_wall_ms = lookup_wall_ms
            self._last_builder_wall_ms = builder_wall_ms
            self._last_message_build_wall_ms = publish_phases[
                "message_build_wall_ms"
            ]
            self._last_publish_call_wall_ms = publish_phases[
                "publish_call_wall_ms"
            ]
        self._valid_pub.publish(Bool(data=True))

    def _publish_observation(
        self, observation, kinematic_diagnostics, trajectory_diagnostics,
        lidar_message, receipt_sec, fusion_duration_wall_ms,
    ):
        message_build_start_wall = time.monotonic()
        message = ObservationCMessage()
        # Preserve the exact sec/nsec pair from the causal lidar packet.
        message.header.stamp = lidar_message.header.stamp
        message.header.frame_id = observation.frame_id
        message.version = observation.version
        message.valid = True
        message.diagnostics = list(observation.diagnostics)
        self._populate_lookup_diagnostics(
            message, kinematic_diagnostics, trajectory_diagnostics,
            lidar_message, receipt_sec,
        )
        lidar = observation.lidar_surrogate
        with self._lock:
            message.producer_input_queue_lag_wall_sec = float(
                self._last_lidar_queue_lag_wall_sec
            )
            message.producer_source_to_callback_sec = float(
                self._last_lidar_source_to_callback_sec
            )
            message.producer_fusion_duration_wall_ms = float(
                fusion_duration_wall_ms
            )
            message.producer_input_count = int(self._lidar_input_count)
            message.producer_processed_count = int(self._lidar_processed_count)
            message.producer_coalesced_drop_count = int(
                self._lidar_coalesced_drop_count
            )
        message.lidar_surrogate = lidar.surrogate.tolist()
        message.lidar_valid_mask = lidar.valid_mask.tolist()
        message.lidar_unknown_mask = lidar.unknown_mask.tolist()
        message.lidar_semantic = lidar.semantic.tolist()
        trajectory = observation.future_trajectory
        message.future_positions_body = [
            Point(x=float(point[0]), y=float(point[1]), z=float(point[2]))
            for point in trajectory.positions_body
        ]
        message.future_sample_offsets = trajectory.sample_offsets.tolist()
        message.trajectory_sampling_mode = trajectory.sampling_mode
        message.trajectory_sample_spacing = trajectory.sample_spacing
        message.trajectory_max_distance = trajectory.max_distance
        message.trajectory_id = trajectory.trajectory_id
        message.trajectory_start_time = rospy.Time.from_sec(
            trajectory.trajectory_start_time_sec
        )
        message.trajectory_source_frame = trajectory.source_frame_id
        state = observation.system_state
        message.actual_velocity_body = Vector3(*state.actual_velocity_body.tolist())
        message.tracking_error_body = Vector3(*state.tracking_error_body.tolist())
        message.tracking_error_norm = state.tracking_error_norm
        message.previous_v_max = state.previous_v_max
        message_build_wall_ms = (
            time.monotonic() - message_build_start_wall
        ) * 1000.0
        publish_start_wall = time.monotonic()
        self._observation_pub.publish(message)
        return {
            "message_build_wall_ms": message_build_wall_ms,
            "publish_call_wall_ms": (
                time.monotonic() - publish_start_wall
            ) * 1000.0,
        }

    def _diagnostic_callback(self, _event):
        now = rospy.Time.now().to_sec()
        with self._lock:
            observation = self._last_observation
            receive_time = self._last_observation_receive_sec
            failure = self._last_failure
            trajectory = self._trajectory_store.current
            latest_v_max = self._v_max_buffer.latest
            previous_v_max = None if latest_v_max is None else latest_v_max[1]
            pending = len(self._pending_lidar) + int(
                self._lidar_mailbox.pending
            )
            generation = self._temporal_generation
            barrier_stamp = self._reset_barrier_stamp
            lidar_input_count = self._lidar_input_count
            lidar_processed_count = self._lidar_processed_count
            lidar_coalesced_drop_count = self._lidar_coalesced_drop_count
            lidar_pose_wait_replace_count = self._lidar_pose_wait_replace_count
            lidar_worker_busy = self._lidar_worker_busy
            lidar_worker_started_wall = self._lidar_worker_started_wall_sec
            lidar_queue_lag = self._last_lidar_queue_lag_wall_sec
            lidar_queue_lag_max = self._max_lidar_queue_lag_wall_sec
            source_to_callback = self._last_lidar_source_to_callback_sec
            worker_total_ms = self._last_lidar_worker_total_wall_ms
            worker_total_max_ms = self._max_lidar_worker_total_wall_ms
            lookup_wall_ms = self._last_lookup_wall_ms
            builder_wall_ms = self._last_builder_wall_ms
            message_build_wall_ms = self._last_message_build_wall_ms
            publish_call_wall_ms = self._last_publish_call_wall_ms
        worker_busy_age = (
            0.0
            if lidar_worker_started_wall is None
            else max(0.0, time.monotonic() - lidar_worker_started_wall)
        )
        age = math.inf if receive_time is None else max(0.0, now - receive_time)
        valid = observation is not None and not failure and age <= self._maximum_input_age
        if observation is not None and age > self._maximum_input_age:
            failure = "observation_stale"
            self._valid_pub.publish(Bool(data=False))
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/trajectory_fusion"
        status.hardware_id = "EGO+{}+Mid360(read-only)".format(
            self._state_source_type
        )
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        status.message = "observation_c_ready" if valid else failure or "not_ready"
        values = {
            "contract_version": OBSERVATION_C_VERSION,
            "control_output": "none",
            "policy_backend": "none",
            "state_source_type": self._state_source_type,
            "state_contract_version": self._state_contract_version,
            "state_velocity_source": self._state_velocity_source,
            "state_lookup_policy": self._state_lookup_policy,
            "require_lidar_state_provenance": self._require_lidar_state_provenance,
            "observation_stamp_sec": "" if observation is None else "{:.9f}".format(observation.stamp_sec),
            "output_frame": self._body_frame,
            "trajectory_source_frame": self._world_frame,
            "trajectory_id": "" if trajectory is None else trajectory.trajectory_id,
            "trajectory_start_time_sec": "" if trajectory is None else "{:.9f}".format(trajectory.start_time_sec),
            "sampling_mode": self._builder.sampler.config.sampling_mode,
            "sample_count": self._builder.sampler.config.sample_count,
            "sample_spacing": self._builder.sampler.config.sample_spacing,
            "max_distance": self._builder.sampler.config.max_distance,
            "pose_lookup": self._pose_lookup_mode,
            "previous_v_max": "" if previous_v_max is None else "{:.6f}".format(previous_v_max),
            "pending_lidar": pending,
            "lidar_input_count": lidar_input_count,
            "lidar_processed_count": lidar_processed_count,
            "lidar_coalesced_drop_count": lidar_coalesced_drop_count,
            "lidar_pose_wait_replace_count": lidar_pose_wait_replace_count,
            "lidar_worker_busy": lidar_worker_busy,
            "lidar_worker_busy_age_wall_sec": "{:.6f}".format(
                worker_busy_age
            ),
            "lidar_worker_total_wall_ms": "{:.6f}".format(worker_total_ms),
            "lidar_worker_total_wall_max_ms": "{:.6f}".format(
                worker_total_max_ms
            ),
            "lidar_lookup_wall_ms": "{:.6f}".format(lookup_wall_ms),
            "lidar_builder_wall_ms": "{:.6f}".format(builder_wall_ms),
            "lidar_message_build_wall_ms": "{:.6f}".format(
                message_build_wall_ms
            ),
            "lidar_publish_call_wall_ms": "{:.6f}".format(
                publish_call_wall_ms
            ),
            "lidar_queue_lag_wall_sec": "{:.6f}".format(lidar_queue_lag),
            "lidar_queue_lag_wall_max_sec": "{:.6f}".format(
                lidar_queue_lag_max
            ),
            "lidar_source_to_callback_sec": "{:.6f}".format(
                source_to_callback
            ),
            "accepted_replans": self._accepted_replans,
            "rejected_trajectories": self._rejected_trajectories,
            "ros_time_reset_count": self._time_reset_count,
            "temporal_generation": generation,
            "reset_barrier_stamp_sec": "{:.9f}".format(barrier_stamp),
            "pre_barrier_state_reject_count": self._pre_barrier_state_reject_count,
            "pre_barrier_lidar_reject_count": self._pre_barrier_lidar_reject_count,
            "pre_barrier_trajectory_reject_count": self._pre_barrier_trajectory_reject_count,
            "pre_barrier_v_max_reject_count": self._pre_barrier_v_max_reject_count,
            "generation_mismatch_count": self._generation_mismatch_count,
            "future_state_use_count": self._future_state_use_count,
            "future_trajectory_use_count": self._future_trajectory_use_count,
            "observation_age_sec": "inf" if not math.isfinite(age) else "{:.6f}".format(age),
        }
        status.values = [_key_value(key, value) for key, value in values.items()]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self._diagnostic_pub.publish(array)


if __name__ == "__main__":
    rospy.init_node("learning_speed_observation_c")
    try:
        ObservationCNode()
        rospy.spin()
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal("Observation C startup failed: %s", error)
        raise
