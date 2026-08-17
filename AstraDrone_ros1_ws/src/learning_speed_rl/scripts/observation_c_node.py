#!/usr/bin/env python3
"""Read-only Scheme C feature fusion; never publishes policy or control output."""

import math
import threading
from collections import deque

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from traj_utils.msg import Bspline

from learning_speed_rl.msg import (
    LidarSurrogateStamped,
    ObservationC as ObservationCMessage,
)
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
        sampling = TrajectorySamplingConfig(
            sample_count=int(param("trajectory_sample_count", 20)),
            sample_spacing=float(param("trajectory_sample_spacing", 0.25)),
            max_distance=float(param("trajectory_max_distance", 5.0)),
            sampling_mode=str(param("trajectory_sampling_mode", "distance")),
            arc_length_resolution_m=float(param("arc_length_resolution_m", 0.02)),
            arc_length_max_depth=int(param("arc_length_max_depth", 16)),
        )
        self._builder = ObservationCBuilder(
            TrajectorySampler(sampling), self._world_frame, self._body_frame
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
        ):
            raise rospy.ROSInitException("invalid Observation C configuration")

        self._trajectory_store = ActiveTrajectoryStore()
        self._v_max_buffer = TimestampedScalarBuffer(
            int(param("system_state/v_max_buffer_capacity", 100))
        )
        self._lidar_valid = False
        self._pending_lidar = deque(maxlen=int(param("timing/pending_lidar_capacity", 10)))
        self._last_observation = None
        self._last_observation_receive_sec = None
        self._last_failure = "waiting_for_data"
        self._pose_lookup_mode = ""
        self._accepted_replans = 0
        self._rejected_trajectories = 0
        self._time_reset_count = 0

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
            queue_size=2,
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
        self._diagnostic_timer = rospy.Timer(
            rospy.Duration(1.0 / self._diagnostics_rate), self._diagnostic_callback
        )
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

    def _clear_for_time_reset(self):
        self._state_buffer.clear()
        self._pending_lidar.clear()
        self._trajectory_store.clear()
        self._v_max_buffer.clear()
        self._last_observation = None
        self._time_reset_count += 1
        self._set_invalid("ros_time_reset")

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
        pending = []
        with self._lock:
            if self._state_buffer.add_pose(pose, receipt_sec):
                self._clear_for_time_reset()
                return
            while (
                self._pending_lidar
                and self._pending_lidar[0].header.stamp.to_sec() <= stamp + 1.0e-9
            ):
                pending.append(self._pending_lidar.popleft())
        for packet in pending:
            self._process_lidar(packet, allow_pending=False)

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
            self._v_max_buffer.add(rospy.Time.now().to_sec(), value)

    def _lidar_callback(self, message):
        self._process_lidar(message, allow_pending=True)

    def _process_lidar(self, message, allow_pending):
        receipt_sec = rospy.Time.now().to_sec()
        stamp = message.header.stamp.to_sec()
        frame = message.header.frame_id.lstrip("/")
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
        with self._lock:
            state, lookup, kinematic_diagnostics = (
                self._state_buffer.lookup_with_diagnostics(stamp, receipt_sec)
            )
            trajectory, trajectory_diagnostics = (
                self._trajectory_store.lookup_with_diagnostics(stamp)
            )
            previous_v_max, v_max_lookup = self._v_max_buffer.lookup(stamp)
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
                    self._pending_lidar.append(message)
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
        self._publish_observation(
            observation, kinematic_diagnostics, trajectory_diagnostics,
            message, receipt_sec,
        )
        with self._lock:
            self._last_observation = observation
            self._last_observation_receive_sec = receipt_sec
            self._last_failure = ""
            self._pose_lookup_mode = lookup
        self._valid_pub.publish(Bool(data=True))

    def _publish_observation(
        self, observation, kinematic_diagnostics, trajectory_diagnostics,
        lidar_message, receipt_sec,
    ):
        message = ObservationCMessage()
        message.header.stamp = rospy.Time.from_sec(observation.stamp_sec)
        message.header.frame_id = observation.frame_id
        message.version = observation.version
        message.valid = True
        message.diagnostics = list(observation.diagnostics)
        self._populate_lookup_diagnostics(
            message, kinematic_diagnostics, trajectory_diagnostics,
            lidar_message, receipt_sec,
        )
        lidar = observation.lidar_surrogate
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
        self._observation_pub.publish(message)

    def _diagnostic_callback(self, _event):
        now = rospy.Time.now().to_sec()
        with self._lock:
            observation = self._last_observation
            receive_time = self._last_observation_receive_sec
            failure = self._last_failure
            trajectory = self._trajectory_store.current
            latest_v_max = self._v_max_buffer.latest
            previous_v_max = None if latest_v_max is None else latest_v_max[1]
            pending = len(self._pending_lidar)
        age = math.inf if receive_time is None else max(0.0, now - receive_time)
        valid = observation is not None and not failure and age <= self._maximum_input_age
        if observation is not None and age > self._maximum_input_age:
            failure = "observation_stale"
            self._valid_pub.publish(Bool(data=False))
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/trajectory_fusion"
        status.hardware_id = "EGO+FAST-LIO+Mid360(read-only)"
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        status.message = "observation_c_ready" if valid else failure or "not_ready"
        values = {
            "contract_version": OBSERVATION_C_VERSION,
            "control_output": "none",
            "policy_backend": "none",
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
            "accepted_replans": self._accepted_replans,
            "rejected_trajectories": self._rejected_trajectories,
            "ros_time_reset_count": self._time_reset_count,
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
