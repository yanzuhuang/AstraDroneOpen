#!/usr/bin/env python3
"""Read-only Mid360 point-cloud surrogate prototype; no policy/control output."""

import math
import os
import resource
import threading
from collections import deque
from time import perf_counter

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32MultiArray, Header, MultiArrayDimension, UInt8MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from learning_speed_rl.msg import LidarSurrogateStamped

from learning_speed_rl.observation.v2 import (
    AngularPartition,
    AngularPartitionSpec,
    BinSemantic,
    CloudFrame,
    CloudHistoryBuffer,
    FovCoverageConfig,
    HistoricalFovEstimator,
    LidarSurrogateBuilder,
    LidarSurrogateConfig,
    Pose3D,
    PoseBuffer,
    decode_xyz_points,
    preserve_source_stamp,
    sensor_to_body,
)
from learning_speed_rl.observation.v2.lidar_surrogate import voxel_downsample


def _key_value(key, value):
    return KeyValue(key=str(key), value=str(value))


def _array_layout(message, label, size):
    message.layout.dim = [
        MultiArrayDimension(label=label, size=size, stride=size)
    ]


class ObservationV2Node:
    def __init__(self):
        self._lock = threading.RLock()
        root = "~observation_v2/"
        param = lambda name, default=None: rospy.get_param(root + name, default)
        if not bool(param("enabled", True)):
            raise rospy.ROSInitException("Observation v2 is explicitly disabled")

        self._history_frames = int(param("history_frames", 5))
        self._minimum_history_frames = int(param("minimum_history_frames", self._history_frames))
        self._cloud_rate_reference_hz = float(param("cloud_rate_reference_hz", 10.0))
        self._maximum_input_points = int(param("preprocessing/maximum_input_points", 250000))
        self._maximum_cloud_pose_delta = float(param("timing/maximum_cloud_pose_delta_sec", 0.05))
        self._maximum_pose_age = float(param("timing/maximum_pose_age_sec", 0.20))
        self._expected_world_frame = str(param("frames/world", "camera_init")).lstrip("/")
        self._expected_body_frame = str(param("frames/body", "body")).lstrip("/")
        self._sensor_frame = str(param("frames/sensor", "mid360_link")).lstrip("/")
        self._cloud_frame_mode = str(
            param("preprocessing/cloud_frame_mode", "world_registered")
        )
        self._pose_lookup_policy = str(
            param("timing/pose_lookup_policy", "interpolate")
        )
        self._state_source_type = str(
            param("state_source_type", "fast_lio_state_estimate")
        )
        self._state_contract_version = str(
            param(
                "state_contract_version", "astradrone_planning_odometry_v1.0"
            )
        )
        self._voxel_size = float(param("voxel_downsample_m", 0.05))
        self._minimum_range = float(param("preprocessing/minimum_range_m", 0.20))
        self._distance_clip = float(param("distance_clip_m", 10.0))
        self._unknown_offset = float(param("unknown_encoding_offset_m", 20.0))
        self._sensor_translation_body_m = np.asarray(
            param(
                "sensor_extrinsic/translation_body_sensor_m",
                [-0.011, -0.02329, 0.04412],
            ),
            dtype=np.float64,
        )
        self._sensor_rotation_body_xyzw = np.asarray(
            param(
                "sensor_extrinsic/rotation_body_sensor_xyzw",
                [0.0, 0.0, 0.0, 1.0],
            ),
            dtype=np.float64,
        )

        angular_spec = AngularPartitionSpec(
            angular_resolution_deg=float(param("angular_resolution_deg", 4.5)),
            azimuth_min_deg=float(param("angular_partition/azimuth_min_deg", -180.0)),
            azimuth_max_deg=float(param("angular_partition/azimuth_max_deg", 180.0)),
            elevation_min_deg=float(param("angular_partition/elevation_min_deg", -90.0)),
            elevation_max_deg=float(param("angular_partition/elevation_max_deg", 90.0)),
            expected_number_of_bins=int(param("number_of_bins", 3200)),
        )
        self._partition = AngularPartition(angular_spec)
        fov_config = FovCoverageConfig(
            sensor_min_range_m=self._minimum_range,
            distance_clip_m=self._distance_clip,
            radial_sample_step_m=float(param("unknown_estimator/radial_sample_step_m", 0.25)),
            azimuth_min_deg=float(param("unknown_estimator/fov_azimuth_min_deg", -180.0)),
            azimuth_max_deg=float(param("unknown_estimator/fov_azimuth_max_deg", 180.0)),
            elevation_min_deg=float(param("unknown_estimator/fov_elevation_min_deg", -7.2123)),
            elevation_max_deg=float(param("unknown_estimator/fov_elevation_max_deg", 52.1640)),
            occlusion_angular_resolution_deg=float(param("unknown_estimator/occlusion_angular_resolution_deg", 4.5)),
            occlusion_margin_m=float(param("unknown_estimator/occlusion_margin_m", 0.10)),
            sensor_translation_body_m=self._sensor_translation_body_m,
            sensor_rotation_body_xyzw=self._sensor_rotation_body_xyzw,
        )
        self._builder = LidarSurrogateBuilder(
            LidarSurrogateConfig(
                voxel_downsample_m=self._voxel_size,
                sensor_min_range_m=self._minimum_range,
                distance_clip_m=self._distance_clip,
                unknown_encoding_offset_m=self._unknown_offset,
                minimum_history_frames=self._minimum_history_frames,
            ),
            self._partition,
            HistoricalFovEstimator(fov_config),
        )
        self._pose_buffer = PoseBuffer(
            int(param("timing/pose_buffer_capacity", 400)),
            float(param("timing/maximum_pose_interpolation_gap_sec", 0.05)),
        )
        self._cloud_history = CloudHistoryBuffer(self._history_frames)
        self._visualization_enabled = bool(param("visualization/enabled", True))
        self._visualization_stride = int(param("visualization/bin_stride", 1))
        self._ray_width = float(param("visualization/ray_width_m", 0.012))
        self._unknown_ray_length = float(param("visualization/unknown_ray_length_m", 1.0))
        self._maximum_aligned_points = int(param("visualization/maximum_aligned_points", 100000))
        if (
            self._history_frames <= 0
            or self._minimum_history_frames <= 0
            or self._minimum_history_frames > self._history_frames
            or self._maximum_input_points <= 0
            or self._maximum_cloud_pose_delta <= 0.0
            or self._maximum_pose_age <= 0.0
            or self._visualization_stride <= 0
            or self._cloud_frame_mode not in ("world_registered", "sensor_raw")
            or self._pose_lookup_policy not in (
                "interpolate", "causal_at_or_before"
            )
            or (
                self._cloud_frame_mode == "sensor_raw"
                and self._pose_lookup_policy != "causal_at_or_before"
            )
        ):
            raise rospy.ROSInitException("invalid Observation v2 configuration")

        topic = lambda name, default: str(param("topics/" + name, default))
        self._surrogate_pub = rospy.Publisher(topic("surrogate", "learning_speed/observation_v2/surrogate"), Float32MultiArray, queue_size=1)
        self._stamped_pub = rospy.Publisher(topic("stamped", "learning_speed/observation_v2/stamped"), LidarSurrogateStamped, queue_size=1)
        self._valid_pub = rospy.Publisher(topic("valid", "learning_speed/observation_v2/valid"), Bool, queue_size=1, latch=True)
        self._valid_mask_pub = rospy.Publisher(topic("valid_mask", "learning_speed/observation_v2/lidar_valid_mask"), Float32MultiArray, queue_size=1)
        self._unknown_mask_pub = rospy.Publisher(topic("unknown_mask", "learning_speed/observation_v2/unknown_mask"), Float32MultiArray, queue_size=1)
        self._semantic_pub = rospy.Publisher(topic("semantic", "learning_speed/observation_v2/semantic"), UInt8MultiArray, queue_size=1)
        self._diagnostic_pub = rospy.Publisher(topic("diagnostics", "learning_speed/observation_v2/diagnostics"), DiagnosticArray, queue_size=1)
        self._aligned_pub = rospy.Publisher(topic("aligned_history", "learning_speed/observation_v2/aligned_history"), PointCloud2, queue_size=1)
        self._visualization_pub = rospy.Publisher(topic("visualization", "learning_speed/observation_v2/visualization"), MarkerArray, queue_size=1)

        self._last_cloud_stamp = None
        self._last_input_stamp = None
        self._last_odom_stamp = None
        self._last_pose_receive_sec = None
        self._last_cloud_receive_sec = None
        self._last_observation = None
        self._last_failure = "waiting_for_data"
        self._cloud_intervals = deque(maxlen=100)
        self._input_source_intervals = deque(maxlen=100)
        self._output_stamp_intervals = deque(maxlen=100)
        self._output_wall_intervals = deque(maxlen=100)
        self._pending_clouds = deque(maxlen=max(10, 2 * self._history_frames))
        self._pending_cloud_drop_count = 0
        self._last_output_wall = None
        self._last_output_stamp = None
        self._last_timings = {}
        self._input_points = 0
        self._frame_points = 0
        self._last_pose_mode = ""
        self._last_pose_source_stamp = None
        self._reset_count = 0
        self._temporal_generation = 0
        self._reset_barrier_stamp = 0.0
        self._pre_barrier_state_reject_count = 0
        self._pre_barrier_cloud_reject_count = 0
        self._future_pose_use_count = 0
        self._pose_lookup_counts = {"exact": 0, "interpolated": 0, "causal_previous": 0}
        self._process = None
        try:
            import psutil
            self._process = psutil.Process(os.getpid())
            self._process.cpu_percent(interval=None)
        except ImportError:
            pass

        self._odom_sub = rospy.Subscriber(topic("odom", "Odometry"), Odometry, self._odom_callback, queue_size=100)
        self._cloud_sub = rospy.Subscriber(topic("cloud", "stage3/cloud_registered_filtered"), PointCloud2, self._cloud_callback, queue_size=1, buff_size=2 ** 26)
        self._clear_service = rospy.Service(
            "~clear_temporal_history", Trigger, self._clear_temporal_history
        )
        diagnostic_rate = float(param("timing/diagnostics_rate_hz", 2.0))
        self._diagnostic_timer = rospy.Timer(rospy.Duration(1.0 / diagnostic_rate), self._diagnostic_callback)
        self._valid_pub.publish(Bool(data=False))
        rospy.logwarn(
            "Observation v2 read-only prototype active: cloud=%s odom=%s bins=%d mode=%s pose_lookup=%s; no v_max/control publisher exists",
            rospy.resolve_name(topic("cloud", "stage3/cloud_registered_filtered")),
            rospy.resolve_name(topic("odom", "Odometry")),
            angular_spec.number_of_bins,
            self._cloud_frame_mode,
            self._pose_lookup_policy,
        )

    def _clear_for_time_reset(self, reason, barrier_stamp=0.0, new_generation=True):
        self._pose_buffer.clear()
        self._cloud_history.clear()
        self._pending_clouds.clear()
        self._input_source_intervals.clear()
        self._cloud_intervals.clear()
        self._output_stamp_intervals.clear()
        self._last_input_stamp = None
        self._last_cloud_stamp = None
        self._last_odom_stamp = None
        self._last_pose_receive_sec = None
        self._last_cloud_receive_sec = None
        self._last_output_stamp = None
        self._last_output_wall = None
        self._last_pose_source_stamp = None
        self._last_observation = None
        self._last_failure = reason
        self._reset_count += 1
        if new_generation:
            self._temporal_generation += 1
        self._reset_barrier_stamp = float(barrier_stamp)
        self._valid_pub.publish(Bool(data=False))

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
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            return
        world = message.header.frame_id.lstrip("/")
        body = message.child_frame_id.lstrip("/")
        if world != self._expected_world_frame or body != self._expected_body_frame:
            with self._lock:
                self._last_failure = "odom_frame_mismatch:{}->{},expected:{}->{}".format(world, body, self._expected_world_frame, self._expected_body_frame)
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
            with self._lock:
                self._last_failure = "invalid_odometry:{}".format(error)
            return
        pending = []
        with self._lock:
            if stamp <= self._reset_barrier_stamp + 1.0e-9:
                self._pre_barrier_state_reject_count += 1
                return
            if self._pose_buffer.add(pose):
                self._clear_for_time_reset(
                    "ros_time_reset_on_odometry", barrier_stamp=0.0,
                    new_generation=True,
                )
                self._pose_buffer.add(pose)
            self._last_odom_stamp = stamp
            self._last_pose_receive_sec = rospy.Time.now().to_sec()
            while self._pending_clouds and self._pending_clouds[0].header.stamp.to_sec() <= stamp + 1.0e-9:
                pending.append(self._pending_clouds.popleft())
        for cloud in pending:
            self._process_cloud(cloud, allow_pending=False)

    def _decode_points(self, message):
        return decode_xyz_points(message, self._maximum_input_points)

    def _cloud_callback(self, message):
        self._process_cloud(message, allow_pending=True)

    def _lidar_source_rate(self):
        return self._frequency(self._input_source_intervals)

    def _publish_invalid_packet(
        self, stamp, frame, reason, cycle_start,
        input_points=0, finite_points=0, in_range_points=0,
        history_frames=0, source_stamp=None, pose_source_stamp=None,
    ):
        packet = LidarSurrogateStamped()
        packet.header.stamp = (
            preserve_source_stamp(source_stamp)
            if source_stamp is not None
            else rospy.Time.from_sec(max(0.0, float(stamp)))
        )
        packet.header.frame_id = frame or self._expected_body_frame
        packet.version = "lidar_surrogate_v2.0"
        packet.valid = False
        packet.diagnostics = [str(reason)]
        packet.temporal_generation = self._temporal_generation
        packet.reset_barrier_stamp = rospy.Time.from_sec(
            max(0.0, self._reset_barrier_stamp)
        )
        packet.state_source_type = self._state_source_type
        packet.state_contract_version = self._state_contract_version
        packet.cloud_frame_mode = self._cloud_frame_mode
        if pose_source_stamp is not None and pose_source_stamp > 0.0:
            packet.pose_source_stamp = rospy.Time.from_sec(pose_source_stamp)
        packet.pose_used_future = bool(
            pose_source_stamp is not None and pose_source_stamp > stamp + 1.0e-9
        )
        packet.input_points = int(max(0, input_points))
        packet.finite_points = int(max(0, finite_points))
        packet.in_range_points = int(max(0, in_range_points))
        packet.history_frames = int(max(0, history_frames))
        packet.lidar_source_rate_hz = self._lidar_source_rate()
        packet.build_duration_ms = (perf_counter() - cycle_start) * 1000.0
        self._stamped_pub.publish(packet)
        self._valid_pub.publish(Bool(data=False))

    def _process_cloud(self, message, allow_pending):
        cycle_start = perf_counter()
        stamp = message.header.stamp.to_sec()
        cloud_frame = message.header.frame_id.lstrip("/")
        expected_cloud_frame = (
            self._expected_world_frame
            if self._cloud_frame_mode == "world_registered"
            else self._sensor_frame
        )
        input_points = int(message.width * message.height)
        with self._lock:
            if stamp <= self._reset_barrier_stamp + 1.0e-9:
                self._pre_barrier_cloud_reject_count += 1
                self._last_failure = "cloud_at_or_before_reset_barrier"
                self._publish_invalid_packet(
                    stamp, self._expected_body_frame,
                    "cloud_at_or_before_reset_barrier", cycle_start,
                    input_points=input_points, source_stamp=message.header.stamp,
                )
                return
            if self._last_input_stamp is not None and stamp > self._last_input_stamp:
                self._input_source_intervals.append(stamp - self._last_input_stamp)
            if self._last_input_stamp is None or stamp >= self._last_input_stamp:
                self._last_input_stamp = stamp
        if stamp <= 0.0 or cloud_frame != expected_cloud_frame:
            reason = (
                "timestamp_invalid" if stamp <= 0.0
                else "frame_invalid:{}!={}".format(
                    cloud_frame, expected_cloud_frame
                )
            )
            with self._lock:
                self._last_failure = reason
            self._publish_invalid_packet(
                stamp, cloud_frame, reason, cycle_start,
                input_points=input_points,
                source_stamp=message.header.stamp,
            )
            return
        with self._lock:
            if self._last_cloud_stamp is not None and stamp < self._last_cloud_stamp - 1.0e-9:
                self._clear_for_time_reset(
                    "ros_time_reset_on_cloud", barrier_stamp=0.0,
                    new_generation=True,
                )
            if self._pose_lookup_policy == "causal_at_or_before":
                pose, pose_mode = self._pose_buffer.lookup_at_or_before(
                    stamp, self._maximum_cloud_pose_delta
                )
            else:
                pose, pose_mode = self._pose_buffer.lookup(stamp)
            newest_pose_stamp = self._pose_buffer.newest_stamp
        if pose is None:
            with self._lock:
                if (
                    allow_pending
                    and self._pose_lookup_policy == "interpolate"
                    and pose_mode == "cloud_newer_than_pose_history"
                ):
                    if len(self._pending_clouds) == self._pending_clouds.maxlen:
                        self._pending_cloud_drop_count += 1
                    self._pending_clouds.append(message)
                    self._last_failure = "waiting_for_timestamped_pose"
                else:
                    self._last_failure = pose_mode
            self._publish_invalid_packet(
                stamp, self._expected_body_frame,
                "timestamp_sync:{}".format(pose_mode), cycle_start,
                input_points=input_points,
                history_frames=self._cloud_history.size,
                source_stamp=message.header.stamp,
            )
            return
        pose_used_future = pose.stamp_sec > stamp + 1.0e-9
        if self._pose_lookup_policy == "causal_at_or_before":
            pose_sync_invalid = (
                pose_used_future
                or stamp - pose.stamp_sec
                > self._maximum_cloud_pose_delta + 1.0e-9
            )
        else:
            pose_sync_invalid = (
                newest_pose_stamp is None
                or newest_pose_stamp - stamp
                > self._maximum_cloud_pose_delta + 1.0e-9
            )
        if pose_sync_invalid:
            if pose_used_future:
                with self._lock:
                    self._future_pose_use_count += 1
            with self._lock:
                self._last_failure = "cloud_pose_timestamp_mismatch"
            self._publish_invalid_packet(
                stamp, self._expected_body_frame,
                "timestamp_sync:cloud_pose_timestamp_mismatch", cycle_start,
                input_points=input_points,
                history_frames=self._cloud_history.size,
                source_stamp=message.header.stamp,
                pose_source_stamp=pose.stamp_sec,
            )
            return

        decode_start = perf_counter()
        try:
            points_input = self._decode_points(message)
        except (KeyError, ValueError, TypeError) as error:
            reason = "pointcloud_decode_failed:{}".format(error)
            with self._lock:
                self._last_failure = reason
            self._publish_invalid_packet(
                stamp, self._expected_body_frame, reason, cycle_start,
                input_points=input_points,
                history_frames=self._cloud_history.size,
                source_stamp=message.header.stamp,
            )
            return
        decode_ms = (perf_counter() - decode_start) * 1000.0
        finite_points = int(points_input.shape[0])
        if self._cloud_frame_mode == "world_registered":
            points_body = pose.world_to_body(points_input)
        else:
            points_body = sensor_to_body(
                points_input,
                self._sensor_translation_body_m,
                self._sensor_rotation_body_xyzw,
            )
        ranges = np.linalg.norm(points_body, axis=1)
        valid = (
            np.all(np.isfinite(points_body), axis=1)
            & np.isfinite(ranges)
            & (ranges > self._minimum_range)
            & (ranges <= self._distance_clip)
        )
        in_range_points = int(np.count_nonzero(valid))
        downsample_start = perf_counter()
        points_body = voxel_downsample(points_body[valid], self._voxel_size)
        per_frame_downsample_ms = (perf_counter() - downsample_start) * 1000.0
        frame = CloudFrame(
            stamp_sec=stamp,
            source_frame_id=cloud_frame,
            points_body=points_body,
            pose_world_body=pose,
            input_points=input_points,
            valid_points=int(points_body.shape[0]),
            downsample_ms=per_frame_downsample_ms,
            pose_lookup_mode=pose_mode,
        )
        with self._lock:
            reset = self._cloud_history.append(frame)
            frames = self._cloud_history.frames
        if reset:
            with self._lock:
                self._reset_count += 1
        if len(frames) < self._minimum_history_frames:
            with self._lock:
                self._last_cloud_stamp = stamp
                self._last_failure = "warming_history:{}/{}".format(len(frames), self._minimum_history_frames)
                self._input_points = input_points
                self._frame_points = int(points_body.shape[0])
            self._publish_invalid_packet(
                stamp, self._expected_body_frame,
                "insufficient_history:{}/{}".format(
                    len(frames), self._minimum_history_frames
                ),
                cycle_start,
                input_points=input_points,
                finite_points=finite_points,
                in_range_points=in_range_points,
                history_frames=len(frames),
                source_stamp=message.header.stamp,
            )
            return

        try:
            observation, aligned, timings = self._builder.build(frames, self._expected_body_frame)
        except (RuntimeError, ValueError) as error:
            reason = "build_failed:{}".format(error)
            with self._lock:
                self._last_failure = reason
            self._publish_invalid_packet(
                stamp, self._expected_body_frame, reason, cycle_start,
                input_points=input_points,
                finite_points=finite_points,
                in_range_points=in_range_points,
                history_frames=len(frames),
                source_stamp=message.header.stamp,
            )
            return
        total_ms = (perf_counter() - cycle_start) * 1000.0
        timings.update(
            pointcloud_decode_ms=decode_ms,
            per_frame_downsample_ms=per_frame_downsample_ms,
            callback_total_ms=total_ms,
        )
        self._publish_observation(
            observation, aligned,
            source_stamp=message.header.stamp,
            pose_source_stamp=pose.stamp_sec,
            input_points=input_points,
            finite_points=finite_points,
            in_range_points=in_range_points,
            history_frames=len(frames),
            build_duration_ms=total_ms,
        )
        wall_now = perf_counter()
        with self._lock:
            if self._last_cloud_stamp is not None and stamp > self._last_cloud_stamp:
                self._cloud_intervals.append(stamp - self._last_cloud_stamp)
            if self._last_output_wall is not None:
                self._output_wall_intervals.append(wall_now - self._last_output_wall)
            if self._last_output_stamp is not None and stamp > self._last_output_stamp:
                self._output_stamp_intervals.append(stamp - self._last_output_stamp)
            self._last_output_wall = wall_now
            self._last_output_stamp = stamp
            self._last_cloud_stamp = stamp
            self._last_cloud_receive_sec = rospy.Time.now().to_sec()
            self._last_observation = observation
            self._last_failure = ""
            self._last_timings = timings
            self._input_points = input_points
            self._frame_points = int(points_body.shape[0])
            self._last_pose_mode = pose_mode
            self._last_pose_source_stamp = pose.stamp_sec
            self._pose_lookup_counts[pose_mode] = (
                self._pose_lookup_counts.get(pose_mode, 0) + 1
            )
        self._valid_pub.publish(Bool(data=True))

    def _publish_float_array(self, publisher, values, label):
        message = Float32MultiArray(data=values)
        _array_layout(message, label, len(message.data))
        publisher.publish(message)

    def _publish_observation(
        self, observation, aligned, source_stamp, pose_source_stamp,
        input_points, finite_points, in_range_points, history_frames,
        build_duration_ms,
    ):
        surrogate_values = observation.lidar_surrogate.astype(
            np.float32, copy=False
        ).tolist()
        valid_mask_values = observation.lidar_valid_mask.astype(
            np.float32, copy=False
        ).tolist()
        unknown_mask_values = observation.unknown_mask.astype(
            np.float32, copy=False
        ).tolist()
        semantic_values = observation.semantic.astype(
            np.uint8, copy=False
        ).tolist()
        if self._surrogate_pub.get_num_connections() > 0:
            self._publish_float_array(
                self._surrogate_pub, surrogate_values, observation.version
            )
        if self._valid_mask_pub.get_num_connections() > 0:
            self._publish_float_array(
                self._valid_mask_pub, valid_mask_values, "lidar_valid_mask"
            )
        if self._unknown_mask_pub.get_num_connections() > 0:
            self._publish_float_array(
                self._unknown_mask_pub, unknown_mask_values, "unknown_mask"
            )
        if self._semantic_pub.get_num_connections() > 0:
            semantic = UInt8MultiArray(data=semantic_values)
            _array_layout(
                semantic,
                "semantic:0_unknown,1_free,2_obstacle",
                observation.number_of_bins,
            )
            self._semantic_pub.publish(semantic)
        stamped = LidarSurrogateStamped()
        # Preserve the original ROS sec/nsec pair. A float to_sec()/from_sec()
        # round trip was observed to subtract exactly 1 ns in worksite, turning
        # a same-scan state sample into a false 0.10 s interpolation gap.
        stamped.header.stamp = preserve_source_stamp(source_stamp)
        stamped.header.frame_id = observation.frame_id
        stamped.version = observation.version
        stamped.valid = True
        stamped.diagnostics = []
        stamped.temporal_generation = self._temporal_generation
        stamped.reset_barrier_stamp = rospy.Time.from_sec(
            max(0.0, self._reset_barrier_stamp)
        )
        stamped.state_source_type = self._state_source_type
        stamped.state_contract_version = self._state_contract_version
        stamped.cloud_frame_mode = self._cloud_frame_mode
        stamped.pose_source_stamp = rospy.Time.from_sec(pose_source_stamp)
        stamped.pose_used_future = pose_source_stamp > observation.stamp_sec + 1.0e-9
        stamped.lidar_surrogate = surrogate_values
        stamped.lidar_valid_mask = valid_mask_values
        stamped.unknown_mask = unknown_mask_values
        stamped.semantic = semantic_values
        stamped.input_points = int(input_points)
        stamped.finite_points = int(finite_points)
        stamped.in_range_points = int(in_range_points)
        stamped.history_frames = int(history_frames)
        stamped.lidar_source_rate_hz = self._lidar_source_rate()
        stamped.build_duration_ms = float(build_duration_ms)
        self._stamped_pub.publish(stamped)
        if aligned.shape[0] > self._maximum_aligned_points:
            stride = int(math.ceil(float(aligned.shape[0]) / self._maximum_aligned_points))
            aligned = aligned[::stride]
        header = Header(
            stamp=preserve_source_stamp(source_stamp),
            frame_id=observation.frame_id,
        )
        if self._aligned_pub.get_num_connections() > 0:
            self._aligned_pub.publish(
                point_cloud2.create_cloud_xyz32(header, aligned.tolist())
            )
        if (
            self._visualization_enabled
            and self._visualization_pub.get_num_connections() > 0
        ):
            self._visualization_pub.publish(self._make_markers(observation))

    def _make_markers(self, observation):
        directions = self._partition.direction_centers()
        marker_array = MarkerArray()
        definitions = (
            (BinSemantic.UNKNOWN, "unknown", (0.55, 0.20, 0.75, 0.40)),
            (BinSemantic.OBSERVED_FREE, "observed_free", (0.10, 0.85, 0.20, 0.32)),
            (BinSemantic.KNOWN_OBSTACLE, "nearest_obstacle", (0.95, 0.10, 0.05, 0.90)),
        )
        stamp = rospy.Time.from_sec(observation.stamp_sec)
        for marker_id, (semantic_value, namespace, color) in enumerate(definitions):
            marker = Marker()
            marker.header.stamp = stamp
            marker.header.frame_id = observation.frame_id
            marker.ns = namespace
            marker.id = marker_id
            marker.type = Marker.LINE_LIST
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = self._ray_width
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
            indices = np.flatnonzero(observation.semantic == int(semantic_value))[:: self._visualization_stride]
            for index in indices:
                if semantic_value == BinSemantic.KNOWN_OBSTACLE:
                    length = observation.nearest_obstacle_distance[index]
                elif semantic_value == BinSemantic.OBSERVED_FREE:
                    length = observation.observed_free_range[index]
                else:
                    length = max(self._unknown_ray_length, observation.observed_free_range[index])
                marker.points.append(Point(x=0.0, y=0.0, z=0.0))
                endpoint = directions[index] * float(length)
                marker.points.append(Point(x=endpoint[0], y=endpoint[1], z=endpoint[2]))
            marker_array.markers.append(marker)
        return marker_array

    @staticmethod
    def _frequency(intervals):
        return 0.0 if not intervals else 1.0 / (sum(intervals) / len(intervals))

    def _diagnostic_callback(self, _event):
        now = rospy.Time.now().to_sec()
        with self._lock:
            observation = self._last_observation
            failure = self._last_failure
            pose_age = math.inf if self._last_odom_stamp is None else max(0.0, now - self._last_odom_stamp)
            cloud_age = math.inf if self._last_cloud_stamp is None else max(0.0, now - self._last_cloud_stamp)
            timings = dict(self._last_timings)
            cloud_rate = self._frequency(self._cloud_intervals)
            output_stamp_rate = self._frequency(self._output_stamp_intervals)
            output_wall_rate = self._frequency(self._output_wall_intervals)
            history_size = self._cloud_history.size
            pending_clouds = len(self._pending_clouds)
            generation = self._temporal_generation
            barrier_stamp = self._reset_barrier_stamp
            pose_lookup_counts = dict(self._pose_lookup_counts)
        stale = pose_age > self._maximum_pose_age
        valid = observation is not None and not failure and not stale
        if stale:
            failure = "pose_stale"
            self._valid_pub.publish(Bool(data=False))
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/lidar_surrogate"
        status.hardware_id = "Mid360+{}(read-only)".format(
            self._state_source_type
        )
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.WARN
        status.message = "observation_v2_ready" if valid else failure or "not_ready"
        metadata = {} if observation is None else observation.metadata
        memory_mib = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
        cpu_percent = float("nan") if self._process is None else self._process.cpu_percent(interval=None)
        values = {
            "contract_version": "lidar_surrogate_v2.0",
            "control_output": "none",
            "state_source_type": self._state_source_type,
            "state_contract_version": self._state_contract_version,
            "observation_stamp_sec": "" if observation is None else "{:.9f}".format(observation.stamp_sec),
            "number_of_bins": self._partition.spec.number_of_bins,
            "bin_order": "elevation_major_azimuth_fast",
            "source_cloud_frame": (
                self._expected_world_frame
                if self._cloud_frame_mode == "world_registered"
                else self._sensor_frame
            ),
            "cloud_frame_mode": self._cloud_frame_mode,
            "output_frame": self._expected_body_frame,
            "sensor_frame": self._sensor_frame,
            "sensor_translation_body_m": self._sensor_translation_body_m.tolist(),
            "sensor_rotation_body_xyzw": self._sensor_rotation_body_xyzw.tolist(),
            "pose_lookup_policy": self._pose_lookup_policy,
            "pose_lookup": self._last_pose_mode,
            "pose_source_stamp_sec": (
                "" if self._last_pose_source_stamp is None
                else "{:.9f}".format(self._last_pose_source_stamp)
            ),
            "pose_exact_count": pose_lookup_counts.get("exact", 0),
            "pose_interpolated_count": pose_lookup_counts.get("interpolated", 0),
            "pose_causal_previous_count": pose_lookup_counts.get("causal_previous", 0),
            "future_pose_use_count": self._future_pose_use_count,
            "history_frames": history_size,
            "pending_clouds": pending_clouds,
            "pending_cloud_drop_count": self._pending_cloud_drop_count,
            "raw_cloud_hz": "{:.3f}".format(self._lidar_source_rate()),
            "accepted_cloud_hz": "{:.3f}".format(cloud_rate),
            "input_cloud_hz": "{:.3f}".format(cloud_rate),
            "output_stamp_hz": "{:.3f}".format(output_stamp_rate),
            "output_wall_hz": "{:.3f}".format(output_wall_rate),
            "cloud_rate_reference_hz": self._cloud_rate_reference_hz,
            "pose_age_sec": "{:.6f}".format(pose_age),
            "cloud_age_sec": "{:.6f}".format(cloud_age),
            "input_points": self._input_points,
            "per_frame_points": self._frame_points,
            "known_obstacle_bins": metadata.get("known_obstacle_bins", 0),
            "observed_free_bins": metadata.get("observed_free_bins", 0),
            "unknown_bins": metadata.get("unknown_bins", self._partition.spec.number_of_bins),
            "pointcloud_decode_ms": "{:.3f}".format(timings.get("pointcloud_decode_ms", 0.0)),
            "history_alignment_ms": "{:.3f}".format(timings.get("history_alignment_ms", 0.0)),
            "per_frame_downsample_ms": "{:.3f}".format(timings.get("per_frame_downsample_ms", 0.0)),
            "fusion_downsample_ms": "{:.3f}".format(timings.get("fusion_downsample_ms", 0.0)),
            "angular_binning_ms": "{:.3f}".format(timings.get("angular_binning_ms", 0.0)),
            "unknown_estimation_ms": "{:.3f}".format(timings.get("unknown_estimation_ms", 0.0)),
            "observation_total_ms": "{:.3f}".format(timings.get("callback_total_ms", 0.0)),
            "process_cpu_percent": "{:.2f}".format(cpu_percent),
            "process_max_rss_mib": "{:.2f}".format(memory_mib),
            "ros_time_reset_count": self._reset_count,
            "temporal_generation": generation,
            "reset_barrier_stamp_sec": "{:.9f}".format(barrier_stamp),
            "pre_barrier_state_reject_count": self._pre_barrier_state_reject_count,
            "pre_barrier_cloud_reject_count": self._pre_barrier_cloud_reject_count,
            "unknown_algorithm": "historical_fov_radial_sampling_v1_approximation",
        }
        status.values = [_key_value(key, value) for key, value in values.items()]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [status]
        self._diagnostic_pub.publish(array)


if __name__ == "__main__":
    rospy.init_node("learning_speed_observation_v2")
    try:
        ObservationV2Node()
        rospy.spin()
    except (ValueError, rospy.ROSException) as error:
        rospy.logfatal("Observation v2 startup failed: %s", error)
        raise
