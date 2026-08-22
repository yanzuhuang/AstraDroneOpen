#!/usr/bin/env python3
"""One-shot runtime qualification instrument for the Hector EGO backend."""

import csv
from collections import deque
import json
import math
import os
import statistics
import threading
import time

import rospy
import rosgraph
import sensor_msgs.point_cloud2 as point_cloud2
from diagnostic_msgs.msg import DiagnosticArray
from controller_manager_msgs.srv import (
    SwitchController,
    SwitchControllerRequest,
)
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import PoseStamped, TwistStamped, WrenchStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Empty as EmptyMessage, Float64, Header, String
from std_srvs.srv import Empty, Trigger
from traj_utils.msg import Bspline

from learning_speed_rl.msg import LidarSurrogateStamped, ObservationC

from hector_ego_training_backend.truth_odom_contract import (
    BACKEND_MODE,
    CONTRACT_VERSION,
    SOURCE_TYPE,
    vector_difference_norm,
    wrapped_angle_difference,
    yaw_from_xyzw,
)


def _norm3(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def _roll_pitch(quaternion):
    sinr = 2.0 * (
        quaternion.w * quaternion.x + quaternion.y * quaternion.z
    )
    cosr = 1.0 - 2.0 * (
        quaternion.x * quaternion.x + quaternion.y * quaternion.y
    )
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (
        quaternion.w * quaternion.y - quaternion.z * quaternion.x
    )
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


def _percentile(values, percentile):
    if not values:
        return math.nan
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile
    lower = int(math.floor(index))
    upper = int(math.ceil(index))
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _finite_or_none(value):
    return value if value is not None and math.isfinite(value) else None


class HectorBackendQualifier:
    FIELDNAMES = (
        "wall_time",
        "sim_time",
        "phase",
        "desired_stamp",
        "trajectory_id",
        "desired_px",
        "desired_py",
        "desired_pz",
        "desired_vx",
        "desired_vy",
        "desired_vz",
        "desired_ax",
        "desired_ay",
        "desired_az",
        "desired_yaw",
        "desired_yaw_dot",
        "actual_px",
        "actual_py",
        "actual_pz",
        "actual_vx",
        "actual_vy",
        "actual_vz",
        "actual_speed",
        "actual_roll",
        "actual_pitch",
        "wrench_fx",
        "wrench_fy",
        "wrench_fz",
        "wrench_tx",
        "wrench_ty",
        "wrench_tz",
        "position_error",
        "velocity_error",
        "backend_mode",
        "backend_ready",
        "pose_controller_state",
        "twist_controller_state",
        "raw_receipt_ros_time",
        "adapted_receipt_ros_time",
        "raw_receipt_monotonic",
        "adapted_receipt_monotonic",
        "relay_receipt_delta_ros_sec",
        "relay_receipt_delta_wall_sec",
        "source_to_raw_receipt_sec",
        "source_to_adapted_receipt_sec",
        "raw_frame_id",
        "raw_child_frame_id",
        "adapted_frame_id",
        "adapted_child_frame_id",
        "raw_px",
        "raw_py",
        "raw_pz",
        "raw_vx",
        "raw_vy",
        "raw_vz",
        "raw_yaw",
        "adapted_yaw",
        "truth_position_difference_m",
        "truth_velocity_difference_mps",
        "truth_yaw_difference_rad",
        "truth_stamp_equal",
    )

    def __init__(self):
        self._lock = threading.RLock()
        self._output_dir = rospy.get_param("~output_dir", "")
        self._qualification_only = bool(
            rospy.get_param("~qualification_only", True)
        )
        if not self._qualification_only:
            raise ValueError("qualification_only must be true")
        if not self._output_dir:
            raise ValueError("~output_dir is required")
        os.makedirs(self._output_dir, exist_ok=True)

        self._frame = rospy.get_param("~frame_id", "world")
        self._body_frame = rospy.get_param("~body_frame_id", "base_link")
        self._raw_truth_topic = rospy.get_param(
            "~raw_truth_topic", "/ground_truth/state"
        )
        self._adapted_odom_topic = rospy.get_param(
            "~adapted_odom_topic", "/uav1/Odometry"
        )
        self._model_name = rospy.get_param("~model_name", "hector_uav1")
        self._require_training_observation_c = bool(
            rospy.get_param("~require_training_observation_c", False)
        )
        self._observation_v2_clear_service = rospy.get_param(
            "~observation_v2_clear_service",
            "/uav1/observation_v2/clear_temporal_history",
        )
        self._observation_c_clear_service = rospy.get_param(
            "~observation_c_clear_service",
            "/uav1/observation_c/clear_temporal_history",
        )
        self._reset_pose = (
            float(rospy.get_param("~reset_hover_x", 0.0)),
            float(rospy.get_param("~reset_hover_y", 0.0)),
            float(rospy.get_param("~reset_hover_z", 1.5)),
            float(rospy.get_param("~reset_hover_yaw", 0.0)),
        )
        self._reset_count = int(rospy.get_param("~reset_count", 20))
        self._hover_duration = float(rospy.get_param("~hover_duration", 5.0))
        self._phase_timeout = float(rospy.get_param("~phase_timeout", 18.0))
        self._reset_ready_timeout = float(
            rospy.get_param("~reset_ready_timeout", 6.0)
        )
        self._goal_tolerance = float(
            rospy.get_param("~position_goal_tolerance", 0.25)
        )
        self._velocity_tolerance = float(
            rospy.get_param("~velocity_ready_tolerance", 0.15)
        )
        self._thresholds = {
            name: float(rospy.get_param("~" + name))
            for name in (
                "hover_max_position_error",
                "hover_max_speed",
                "tracking_p95_position_error",
                "tracking_max_position_error",
                "tracking_p95_velocity_error",
                "hold_max_speed",
                "reset_max_position_error",
                "reset_max_speed",
                "reset_max_roll_pitch",
                "minimum_nonideal_error",
                "truth_max_position_difference_m",
                "truth_max_velocity_difference_mps",
                "truth_max_yaw_difference_rad",
                "truth_relay_latency_p95_sec",
                "minimum_truth_pair_ratio",
            )
        }
        self._minimum_replans = int(rospy.get_param("~minimum_replans", 2))
        if self._reset_count < 20:
            raise ValueError("reset_count must be at least 20")

        self._phase = "startup"
        self._desired = None
        self._odom = None
        self._wrench = None
        self._position_error = math.nan
        self._velocity_error = math.nan
        self._backend = {}
        self._truth_state = {}
        self._samples = []
        self._phase_summaries = {}
        self._reset_results = []
        self._events = []
        self._counts = {
            "position_command": 0,
            "pose_command": 0,
            "twist_command": 0,
            "raw_truth": 0,
            "adapted_odometry": 0,
            "wrench": 0,
            "bspline": 0,
        }
        self._bspline_ids = []
        self._last_odom_stamp = rospy.Time(0)
        self._last_raw_stamp = rospy.Time(0)
        self._last_pair_row = None
        self._raw_messages = {}
        self._adapted_messages = {}
        self._raw_keys = deque(maxlen=1000)
        self._adapted_keys = deque(maxlen=1000)
        self._recording_open = True
        self._raw_mid360_samples = []
        self._v2_samples = []
        self._observation_c_samples = []
        self._v2_diagnostics = {}
        self._observation_c_diagnostics = {}
        self._last_surrogate_by_phase = {}
        self._surrogate_variation_by_phase = {}

        self._csv_file = open(
            os.path.join(self._output_dir, "tracking_samples.csv"),
            "w",
            newline="",
            encoding="utf-8",
        )
        self._writer = csv.DictWriter(self._csv_file, fieldnames=self.FIELDNAMES)
        self._writer.writeheader()

        self._goal_pub = rospy.Publisher(
            "/uav1/planning/goal", PoseStamped, queue_size=1
        )
        self._cancel_pub = rospy.Publisher(
            "/uav1/planning/cancel", EmptyMessage, queue_size=1
        )
        self._cloud_pub = rospy.Publisher(
            "/uav1/qualification/cloud", PointCloud2, queue_size=1
        )

        rospy.Subscriber(
            "/uav1/planning/pos_cmd",
            PositionCommand,
            self._desired_callback,
            queue_size=50,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/uav1/planning/bspline",
            Bspline,
            self._bspline_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            self._raw_truth_topic,
            Odometry,
            self._raw_truth_callback,
            queue_size=100,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self._adapted_odom_topic,
            Odometry,
            self._adapted_odom_callback,
            queue_size=100,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/command/wrench",
            WrenchStamped,
            self._wrench_callback,
            queue_size=100,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/command/pose", PoseStamped, self._pose_count_callback, queue_size=100
        )
        rospy.Subscriber(
            "/command/twist",
            TwistStamped,
            self._twist_count_callback,
            queue_size=100,
        )
        rospy.Subscriber(
            "/uav1/position_command_to_hector/position_tracking_error",
            Float64,
            self._position_error_callback,
            queue_size=100,
        )
        rospy.Subscriber(
            "/uav1/position_command_to_hector/velocity_tracking_error",
            Float64,
            self._velocity_error_callback,
            queue_size=100,
        )
        rospy.Subscriber(
            "/uav1/position_command_to_hector/backend_state",
            String,
            self._backend_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            "/uav1/training/odometry/state",
            String,
            self._truth_state_callback,
            queue_size=10,
        )
        if self._require_training_observation_c:
            rospy.Subscriber(
                "/uav1/livox/lidar", PointCloud2,
                self._raw_mid360_callback, queue_size=20,
            )
            rospy.Subscriber(
                "/uav1/learning_speed/observation_v2/stamped",
                LidarSurrogateStamped, self._v2_callback, queue_size=20,
            )
            rospy.Subscriber(
                "/uav1/learning_speed/observation_c",
                ObservationC, self._observation_c_callback, queue_size=20,
            )
            rospy.Subscriber(
                "/uav1/learning_speed/observation_v2/diagnostics",
                DiagnosticArray, self._v2_diagnostic_callback, queue_size=10,
            )
            rospy.Subscriber(
                "/uav1/learning_speed/observation_c/diagnostics",
                DiagnosticArray, self._observation_c_diagnostic_callback,
                queue_size=10,
            )

        self._switch_controller = rospy.ServiceProxy(
            "/controller_manager/switch_controller", SwitchController
        )
        self._set_model_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )
        self._pause_physics = rospy.ServiceProxy("/gazebo/pause_physics", Empty)
        self._unpause_physics = rospy.ServiceProxy(
            "/gazebo/unpause_physics", Empty
        )
        self._adapter_hold = rospy.ServiceProxy(
            "/uav1/position_command_to_hector/hold", Trigger
        )
        self._adapter_prepare_reset = rospy.ServiceProxy(
            "/uav1/position_command_to_hector/prepare_reset", Trigger
        )
        self._adapter_resume_reset = rospy.ServiceProxy(
            "/uav1/position_command_to_hector/resume_reset_hover", Trigger
        )
        self._adapter_engage = rospy.ServiceProxy(
            "/uav1/position_command_to_hector/engage", Trigger
        )
        self._observation_v2_clear = rospy.ServiceProxy(
            self._observation_v2_clear_service, Trigger
        )
        self._observation_c_clear = rospy.ServiceProxy(
            self._observation_c_clear_service, Trigger
        )

        self._cloud_timer = rospy.Timer(
            rospy.Duration(0.1), self._cloud_timer_callback
        )
        self._overall_wall_start = time.monotonic()
        self._overall_sim_start = rospy.Time.now().to_sec()

    def _event(self, name, **fields):
        event = {
            "name": name,
            "wall_time": time.time(),
            "sim_time": rospy.Time.now().to_sec(),
        }
        event.update(fields)
        self._events.append(event)
        rospy.logwarn("[HECTOR QUALIFICATION] %s %s", name, fields)

    def _desired_callback(self, message):
        with self._lock:
            self._desired = message
            self._counts["position_command"] += 1

    def _bspline_callback(self, message):
        with self._lock:
            self._counts["bspline"] += 1
            self._bspline_ids.append((self._phase, int(message.traj_id)))

    def _wrench_callback(self, message):
        with self._lock:
            self._wrench = message
            self._counts["wrench"] += 1

    def _pose_count_callback(self, _message):
        with self._lock:
            self._counts["pose_command"] += 1

    def _twist_count_callback(self, _message):
        with self._lock:
            self._counts["twist_command"] += 1

    def _position_error_callback(self, message):
        with self._lock:
            self._position_error = float(message.data)

    def _velocity_error_callback(self, message):
        with self._lock:
            self._velocity_error = float(message.data)

    def _backend_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._backend = payload

    def _truth_state_callback(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._truth_state = payload

    @staticmethod
    def _diagnostic_values(message):
        if not message.status:
            return {}
        return {item.key: item.value for item in message.status[0].values}

    def _v2_diagnostic_callback(self, message):
        with self._lock:
            self._v2_diagnostics = self._diagnostic_values(message)

    def _observation_c_diagnostic_callback(self, message):
        with self._lock:
            self._observation_c_diagnostics = self._diagnostic_values(message)

    def _raw_mid360_callback(self, message):
        with self._lock:
            self._raw_mid360_samples.append(
                {
                    "phase": self._phase,
                    "stamp": message.header.stamp.to_sec(),
                    "receipt": rospy.Time.now().to_sec(),
                    "frame": message.header.frame_id.lstrip("/"),
                    "points": int(message.width * message.height),
                }
            )

    def _v2_callback(self, message):
        surrogate = [float(value) for value in message.lidar_surrogate]
        semantic = (
            list(message.semantic)
            if not isinstance(message.semantic, (bytes, bytearray))
            else list(bytearray(message.semantic))
        )
        finite = bool(surrogate) and all(math.isfinite(value) for value in surrogate)
        with self._lock:
            phase = self._phase
            previous = self._last_surrogate_by_phase.get(phase)
            variation = None
            if previous is not None and len(previous) == len(surrogate):
                variation = statistics.mean(
                    abs(current - old)
                    for current, old in zip(surrogate, previous)
                )
                self._surrogate_variation_by_phase.setdefault(phase, []).append(
                    variation
                )
            if message.valid:
                self._last_surrogate_by_phase[phase] = surrogate
            self._v2_samples.append(
                {
                    "phase": phase,
                    "stamp": message.header.stamp.to_sec(),
                    "receipt": rospy.Time.now().to_sec(),
                    "valid": bool(message.valid),
                    "generation": int(message.temporal_generation),
                    "barrier": message.reset_barrier_stamp.to_sec(),
                    "state_source": message.state_source_type,
                    "state_contract": message.state_contract_version,
                    "cloud_frame_mode": message.cloud_frame_mode,
                    "pose_stamp": message.pose_source_stamp.to_sec(),
                    "pose_used_future": bool(message.pose_used_future),
                    "history_frames": int(message.history_frames),
                    "bins": len(surrogate),
                    "finite": finite,
                    "all_zero": bool(surrogate) and all(abs(value) <= 1.0e-12 for value in surrogate),
                    "unknown_bins": semantic.count(0),
                    "free_bins": semantic.count(1),
                    "obstacle_bins": semantic.count(2),
                    "variation_l1_mean": variation,
                }
            )

    def _observation_c_callback(self, message):
        lidar = [float(value) for value in message.lidar_surrogate]
        future_dimension = 3 * len(message.future_positions_body)
        policy_dimension = len(lidar) + future_dimension + 3 + 3 + 1
        velocity = message.actual_velocity_body
        tracking = message.tracking_error_body
        finite = (
            all(math.isfinite(value) for value in lidar)
            and all(
                math.isfinite(value)
                for point in message.future_positions_body
                for value in (point.x, point.y, point.z)
            )
            and all(
                math.isfinite(value)
                for value in (
                    velocity.x, velocity.y, velocity.z,
                    tracking.x, tracking.y, tracking.z,
                    message.previous_v_max,
                )
            )
        )
        with self._lock:
            self._observation_c_samples.append(
                {
                    "phase": self._phase,
                    "stamp": message.header.stamp.to_sec(),
                    "receipt": rospy.Time.now().to_sec(),
                    "valid": bool(message.valid),
                    "generation": int(message.temporal_generation),
                    "lidar_generation": int(message.lidar_temporal_generation),
                    "barrier": message.reset_barrier_stamp.to_sec(),
                    "lidar_pose_stamp": message.lidar_pose_source_stamp.to_sec(),
                    "lidar_pose_used_future": bool(message.lidar_pose_used_future),
                    "state_source": message.state_source_type,
                    "state_contract": message.state_contract_version,
                    "kinematic_lookup": message.kinematic_lookup_result,
                    "state_before_stamp": message.state_before_stamp.to_sec(),
                    "state_after_missing": bool(message.state_after_missing),
                    "state_after_stamp": message.state_after_stamp.to_sec(),
                    "trajectory_id": int(message.trajectory_id),
                    "trajectory_start": message.trajectory_start_time.to_sec(),
                    "policy_dimension": policy_dimension,
                    "lidar_bins": len(lidar),
                    "future_points": len(message.future_positions_body),
                    "finite": finite,
                    "actual_speed": _norm3(velocity.x, velocity.y, velocity.z),
                    "tracking_error": _norm3(tracking.x, tracking.y, tracking.z),
                }
            )

    @staticmethod
    def _stamp_key(message):
        return (message.header.stamp.secs, message.header.stamp.nsecs)

    def _raw_truth_callback(self, message):
        receipt = (rospy.Time.now().to_sec(), time.monotonic())
        with self._lock:
            if not self._recording_open:
                return
            key = self._stamp_key(message)
            self._raw_messages[key] = (message, receipt)
            self._raw_keys.append(key)
            self._last_raw_stamp = message.header.stamp
            self._counts["raw_truth"] += 1
            self._try_record_pair_locked(key)
            self._prune_pairs_locked()

    def _adapted_odom_callback(self, message):
        receipt = (rospy.Time.now().to_sec(), time.monotonic())
        with self._lock:
            if not self._recording_open:
                return
            self._odom = message
            self._last_odom_stamp = message.header.stamp
            key = self._stamp_key(message)
            self._adapted_messages[key] = (message, receipt)
            self._adapted_keys.append(key)
            self._counts["adapted_odometry"] += 1
            self._try_record_pair_locked(key)
            self._prune_pairs_locked()

    def _try_record_pair_locked(self, key):
        if key not in self._raw_messages or key not in self._adapted_messages:
            return
        raw, raw_receipt = self._raw_messages.pop(key)
        adapted, adapted_receipt = self._adapted_messages.pop(key)
        row = self._sample_row_locked(
            raw, adapted, raw_receipt, adapted_receipt
        )
        self._last_pair_row = row
        self._samples.append(row)
        self._writer.writerow(row)
        if len(self._samples) % 100 == 0:
            self._csv_file.flush()

    def _prune_pairs_locked(self):
        while len(self._raw_messages) > 500 and self._raw_keys:
            self._raw_messages.pop(self._raw_keys.popleft(), None)
        while len(self._adapted_messages) > 500 and self._adapted_keys:
            self._adapted_messages.pop(self._adapted_keys.popleft(), None)

    def _sample_row_locked(
        self, raw, odometry, raw_receipt, adapted_receipt
    ):
        desired = self._desired
        wrench = self._wrench
        position = odometry.pose.pose.position
        velocity = odometry.twist.twist.linear
        roll, pitch = _roll_pitch(odometry.pose.pose.orientation)
        raw_position = raw.pose.pose.position
        raw_velocity = raw.twist.twist.linear
        raw_quaternion = raw.pose.pose.orientation
        adapted_quaternion = odometry.pose.pose.orientation
        raw_yaw = yaw_from_xyzw(
            (raw_quaternion.x, raw_quaternion.y, raw_quaternion.z, raw_quaternion.w)
        )
        adapted_yaw = yaw_from_xyzw(
            (
                adapted_quaternion.x,
                adapted_quaternion.y,
                adapted_quaternion.z,
                adapted_quaternion.w,
            )
        )
        row = {name: "" for name in self.FIELDNAMES}
        row.update(
            wall_time=time.time(),
            sim_time=odometry.header.stamp.to_sec(),
            phase=self._phase,
            actual_px=position.x,
            actual_py=position.y,
            actual_pz=position.z,
            actual_vx=velocity.x,
            actual_vy=velocity.y,
            actual_vz=velocity.z,
            actual_speed=_norm3(velocity.x, velocity.y, velocity.z),
            actual_roll=roll,
            actual_pitch=pitch,
            position_error=self._position_error,
            velocity_error=self._velocity_error,
            backend_mode=self._backend.get("mode", ""),
            backend_ready=self._backend.get("ready", False),
            pose_controller_state=self._backend.get(
                "pose_controller_state", ""
            ),
            twist_controller_state=self._backend.get(
                "twist_controller_state", ""
            ),
            raw_receipt_ros_time=raw_receipt[0],
            adapted_receipt_ros_time=adapted_receipt[0],
            raw_receipt_monotonic=raw_receipt[1],
            adapted_receipt_monotonic=adapted_receipt[1],
            relay_receipt_delta_ros_sec=max(
                0.0, adapted_receipt[0] - raw_receipt[0]
            ),
            relay_receipt_delta_wall_sec=max(
                0.0, adapted_receipt[1] - raw_receipt[1]
            ),
            source_to_raw_receipt_sec=max(
                0.0, raw_receipt[0] - raw.header.stamp.to_sec()
            ),
            source_to_adapted_receipt_sec=max(
                0.0, adapted_receipt[0] - odometry.header.stamp.to_sec()
            ),
            raw_frame_id=raw.header.frame_id,
            raw_child_frame_id=raw.child_frame_id,
            adapted_frame_id=odometry.header.frame_id,
            adapted_child_frame_id=odometry.child_frame_id,
            raw_px=raw_position.x,
            raw_py=raw_position.y,
            raw_pz=raw_position.z,
            raw_vx=raw_velocity.x,
            raw_vy=raw_velocity.y,
            raw_vz=raw_velocity.z,
            raw_yaw=raw_yaw,
            adapted_yaw=adapted_yaw,
            truth_position_difference_m=vector_difference_norm(
                (raw_position.x, raw_position.y, raw_position.z),
                (position.x, position.y, position.z),
            ),
            truth_velocity_difference_mps=vector_difference_norm(
                (raw_velocity.x, raw_velocity.y, raw_velocity.z),
                (velocity.x, velocity.y, velocity.z),
            ),
            truth_yaw_difference_rad=abs(
                wrapped_angle_difference(adapted_yaw, raw_yaw)
            ),
            truth_stamp_equal=(raw.header.stamp == odometry.header.stamp),
        )
        if desired is not None:
            row.update(
                desired_stamp=desired.header.stamp.to_sec(),
                trajectory_id=int(desired.trajectory_id),
                desired_px=desired.position.x,
                desired_py=desired.position.y,
                desired_pz=desired.position.z,
                desired_vx=desired.velocity.x,
                desired_vy=desired.velocity.y,
                desired_vz=desired.velocity.z,
                desired_ax=desired.acceleration.x,
                desired_ay=desired.acceleration.y,
                desired_az=desired.acceleration.z,
                desired_yaw=desired.yaw,
                desired_yaw_dot=desired.yaw_dot,
            )
        if wrench is not None:
            row.update(
                wrench_fx=wrench.wrench.force.x,
                wrench_fy=wrench.wrench.force.y,
                wrench_fz=wrench.wrench.force.z,
                wrench_tx=wrench.wrench.torque.x,
                wrench_ty=wrench.wrench.torque.y,
                wrench_tz=wrench.wrench.torque.z,
            )
        return row

    def _cloud_timer_callback(self, _event):
        stamp = rospy.Time.now()
        if stamp.is_zero():
            return
        header = Header(stamp=stamp, frame_id=self._frame)
        points = (
            (-4.0, -4.0, 0.25),
            (-4.0, 4.0, 0.25),
            (4.0, -4.0, 0.25),
            (4.0, 4.0, 0.25),
        )
        self._cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, points))

    def _service_wait(self):
        names = [
            "/controller_manager/switch_controller",
            "/gazebo/set_model_state",
            "/gazebo/pause_physics",
            "/gazebo/unpause_physics",
            "/uav1/position_command_to_hector/hold",
            "/uav1/position_command_to_hector/prepare_reset",
            "/uav1/position_command_to_hector/resume_reset_hover",
            "/uav1/position_command_to_hector/engage",
        ]
        if self._require_training_observation_c:
            names.extend(
                [
                    self._observation_v2_clear_service,
                    self._observation_c_clear_service,
                ]
            )
        for name in names:
            rospy.wait_for_service(name, timeout=30.0)

    def _wait_wall(self, predicate, wall_timeout, description):
        deadline = time.monotonic() + wall_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return True
            time.sleep(0.02)
        self._event("wait_timeout", description=description)
        return False

    def _backend_ready(self):
        with self._lock:
            return bool(self._backend.get("ready", False)) and self._odom is not None

    def _actual_state(self):
        with self._lock:
            if self._odom is None:
                return None
            position = self._odom.pose.pose.position
            velocity = self._odom.twist.twist.linear
            roll, pitch = _roll_pitch(self._odom.pose.pose.orientation)
            return (
                (position.x, position.y, position.z),
                (velocity.x, velocity.y, velocity.z),
                roll,
                pitch,
                self._last_odom_stamp.to_sec(),
            )

    def _set_phase(self, phase):
        with self._lock:
            self._phase = phase

    def _phase_snapshot(self):
        with self._lock:
            return {
                "sample_index": len(self._samples),
                "counts": dict(self._counts),
                "sim": rospy.Time.now().to_sec(),
                "wall": time.monotonic(),
            }

    def _finish_phase(self, phase, start):
        end_sim = rospy.Time.now().to_sec()
        end_wall = time.monotonic()
        with self._lock:
            rows = list(self._samples[start["sample_index"] :])
            counts = dict(self._counts)
            bspline_ids = sorted(
                {trajectory_id for item_phase, trajectory_id in self._bspline_ids if item_phase == phase}
            )
        sim_duration = max(0.0, end_sim - start["sim"])
        wall_duration = max(1.0e-9, end_wall - start["wall"])
        position_errors = [
            float(row["position_error"])
            for row in rows
            if row["position_error"] != ""
            and math.isfinite(float(row["position_error"]))
        ]
        velocity_errors = [
            float(row["velocity_error"])
            for row in rows
            if row["velocity_error"] != ""
            and math.isfinite(float(row["velocity_error"]))
        ]
        speeds = [float(row["actual_speed"]) for row in rows]
        roll_pitch = [
            max(abs(float(row["actual_roll"])), abs(float(row["actual_pitch"])))
            for row in rows
        ]
        truth_position_differences = [
            float(row["truth_position_difference_m"]) for row in rows
        ]
        truth_velocity_differences = [
            float(row["truth_velocity_difference_mps"]) for row in rows
        ]
        truth_yaw_differences = [
            float(row["truth_yaw_difference_rad"]) for row in rows
        ]
        relay_latencies_ros = [
            float(row["relay_receipt_delta_ros_sec"]) for row in rows
        ]
        relay_latencies_wall = [
            float(row["relay_receipt_delta_wall_sec"]) for row in rows
        ]
        tail = rows[len(rows) // 2 :] if rows else []
        summary = {
            "sample_count": len(rows),
            "sim_duration": sim_duration,
            "wall_duration": wall_duration,
            "rtf": sim_duration / wall_duration,
            "position_error_max": max(position_errors) if position_errors else None,
            "position_error_rms": (
                math.sqrt(statistics.mean(value * value for value in position_errors))
                if position_errors
                else None
            ),
            "position_error_p95": _finite_or_none(
                _percentile(position_errors, 0.95)
            ),
            "velocity_error_max": max(velocity_errors) if velocity_errors else None,
            "velocity_error_p95": _finite_or_none(
                _percentile(velocity_errors, 0.95)
            ),
            "actual_speed_max": max(speeds) if speeds else None,
            "tail_speed_max": (
                max(float(row["actual_speed"]) for row in tail) if tail else None
            ),
            "roll_pitch_max": max(roll_pitch) if roll_pitch else None,
            "unique_bspline_ids": bspline_ids,
            "truth_contract": {
                "raw_frames": sorted(
                    {row["raw_frame_id"] for row in rows}
                ),
                "raw_child_frames": sorted(
                    {row["raw_child_frame_id"] for row in rows}
                ),
                "adapted_frames": sorted(
                    {row["adapted_frame_id"] for row in rows}
                ),
                "adapted_child_frames": sorted(
                    {row["adapted_child_frame_id"] for row in rows}
                ),
                "stamp_equal_count": sum(
                    1 for row in rows if row["truth_stamp_equal"]
                ),
                "position_difference_max_m": (
                    max(truth_position_differences)
                    if truth_position_differences else None
                ),
                "velocity_difference_max_mps": (
                    max(truth_velocity_differences)
                    if truth_velocity_differences else None
                ),
                "yaw_difference_max_rad": (
                    max(truth_yaw_differences)
                    if truth_yaw_differences else None
                ),
                "relay_latency_ros_p95_sec": _finite_or_none(
                    _percentile(relay_latencies_ros, 0.95)
                ),
                "relay_latency_wall_p95_sec": _finite_or_none(
                    _percentile(relay_latencies_wall, 0.95)
                ),
            },
            "frequencies_hz": {
                name: (
                    (counts[name] - start["counts"][name]) / sim_duration
                    if sim_duration > 0.0
                    else None
                )
                for name in counts
            },
        }
        self._phase_summaries[phase] = summary
        self._event("phase_complete", phase=phase, summary=summary)
        return summary

    def _backend_isolation_audit(self):
        publishers, subscribers, _ = rosgraph.Master(
            rospy.get_name()
        ).getSystemState()
        publisher_map = {topic: sorted(nodes) for topic, nodes in publishers}
        subscriber_map = {topic: sorted(nodes) for topic, nodes in subscribers}
        all_nodes = sorted(
            {
                node
                for mapping in (publisher_map, subscriber_map)
                for nodes in mapping.values()
                for node in nodes
            }
        )
        odom_publishers = publisher_map.get(self._adapted_odom_topic, [])
        odom_subscribers = subscriber_map.get(self._adapted_odom_topic, [])
        forbidden = [
            node for node in all_nodes
            if any(token in node.lower() for token in (
                "fast_lio", "mavros", "px4", "ego_gazebo_bridge"
            ))
        ]
        ego_subscriber = any(
            "ego_planner" in node for node in odom_subscribers
        )
        hector_adapter_subscriber = any(
            "position_command_to_hector" in node for node in odom_subscribers
        )
        expected_publisher = [
            node for node in odom_publishers
            if "gazebo_truth_odometry_adapter" in node
        ]
        return {
            "adapted_odom_topic": self._adapted_odom_topic,
            "adapted_odom_publishers": odom_publishers,
            "adapted_odom_subscribers": odom_subscribers,
            "exactly_one_truth_adapter_publisher": (
                len(odom_publishers) == 1 and len(expected_publisher) == 1
            ),
            "ego_is_subscriber": ego_subscriber,
            "hector_command_adapter_is_subscriber": hector_adapter_subscriber,
            "forbidden_full_stack_nodes": forbidden,
            "pass": bool(
                len(odom_publishers) == 1
                and len(expected_publisher) == 1
                and ego_subscriber
                and hector_adapter_subscriber
                and not forbidden
            ),
        }

    def _truth_contract_summary(self):
        with self._lock:
            rows = list(self._samples)
            counts = dict(self._counts)
            truth_state = dict(self._truth_state)
        paired = len(rows)
        denominator = max(
            counts.get("raw_truth", 0), counts.get("adapted_odometry", 0), 1
        )
        position = [float(row["truth_position_difference_m"]) for row in rows]
        velocity = [float(row["truth_velocity_difference_mps"]) for row in rows]
        yaw = [float(row["truth_yaw_difference_rad"]) for row in rows]
        relay_ros = [float(row["relay_receipt_delta_ros_sec"]) for row in rows]
        relay_wall = [float(row["relay_receipt_delta_wall_sec"]) for row in rows]
        source_to_adapted = [
            float(row["source_to_adapted_receipt_sec"]) for row in rows
        ]
        return {
            "contract_version": CONTRACT_VERSION,
            "backend_mode": BACKEND_MODE,
            "source_type": SOURCE_TYPE,
            "raw_topic": self._raw_truth_topic,
            "adapted_topic": self._adapted_odom_topic,
            "paired_sample_count": paired,
            "pair_ratio": paired / float(denominator),
            "all_stamps_equal": bool(
                rows and all(row["truth_stamp_equal"] for row in rows)
            ),
            "raw_frames": sorted({row["raw_frame_id"] for row in rows}),
            "raw_child_frames": sorted(
                {row["raw_child_frame_id"] for row in rows}
            ),
            "adapted_frames": sorted(
                {row["adapted_frame_id"] for row in rows}
            ),
            "adapted_child_frames": sorted(
                {row["adapted_child_frame_id"] for row in rows}
            ),
            "position_difference_max_m": max(position) if position else None,
            "velocity_difference_max_mps": max(velocity) if velocity else None,
            "yaw_difference_max_rad": max(yaw) if yaw else None,
            "relay_latency_ros_p50_sec": _finite_or_none(
                _percentile(relay_ros, 0.50)
            ),
            "relay_latency_ros_p95_sec": _finite_or_none(
                _percentile(relay_ros, 0.95)
            ),
            "relay_latency_wall_p50_sec": _finite_or_none(
                _percentile(relay_wall, 0.50)
            ),
            "relay_latency_wall_p95_sec": _finite_or_none(
                _percentile(relay_wall, 0.95)
            ),
            "source_to_adapted_latency_ros_p95_sec": _finite_or_none(
                _percentile(source_to_adapted, 0.95)
            ),
            "adapter_state": truth_state,
        }

    def _sleep_sim(self, duration, wall_timeout=None):
        start_sim = rospy.Time.now().to_sec()
        deadline = time.monotonic() + (
            wall_timeout if wall_timeout is not None else max(10.0, duration * 4.0)
        )
        while not rospy.is_shutdown():
            if rospy.Time.now().to_sec() - start_sim >= duration:
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)
        return False

    def _publish_goal(self, x, y, z):
        if not self._wait_wall(
            lambda: self._goal_pub.get_num_connections() > 0,
            10.0,
            "EGO goal subscriber",
        ):
            return False
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = self._frame
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        self._event("goal", x=x, y=y, z=z)
        return True

    def _run_timed_phase(self, phase, duration, goal=None):
        self._set_phase(phase)
        start = self._phase_snapshot()
        if goal is not None and not self._publish_goal(*goal):
            raise RuntimeError("failed to publish goal for " + phase)
        if not self._sleep_sim(duration, self._phase_timeout * 2.0):
            raise RuntimeError("sim time did not advance for " + phase)
        return self._finish_phase(phase, start)

    def _switch(self, start, stop):
        request = SwitchControllerRequest()
        request.start_controllers = list(start)
        request.stop_controllers = list(stop)
        request.strictness = SwitchControllerRequest.STRICT
        request.start_asap = True
        request.timeout = 2.0
        response = self._switch_controller(request)
        if not response.ok:
            raise RuntimeError(
                "controller switch failed start={} stop={}".format(start, stop)
            )

    def _set_reset_model_state(self):
        x, y, z, yaw = self._reset_pose
        request = SetModelStateRequest()
        state = ModelState()
        state.model_name = self._model_name
        state.reference_frame = self._frame
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.z = math.sin(0.5 * yaw)
        state.pose.orientation.w = math.cos(0.5 * yaw)
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0
        request.model_state = state
        response = self._set_model_state(request)
        if not response.success:
            raise RuntimeError("set_model_state failed: " + response.status_message)

    def _wait_low_speed(self, timeout):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            state = self._actual_state()
            if state is not None and _norm3(*state[1]) <= self._velocity_tolerance:
                return True
            time.sleep(0.02)
        return False

    @staticmethod
    def _parse_generation_barrier(message):
        values = {}
        for item in str(message).split(";"):
            if "=" in item:
                key, value = item.split("=", 1)
                values[key.strip()] = value.strip()
        return int(values["generation"]), float(values["barrier"])

    def _clear_observation_temporal_history(self):
        # Clear the consumer first, then the producer. The producer's later
        # barrier becomes the common effective lower bound for every cloud
        # that Observation C can subsequently receive in this generation.
        c_response = self._observation_c_clear()
        v2_response = self._observation_v2_clear()
        if not c_response.success:
            raise RuntimeError(
                "Observation C reset barrier failed: " + c_response.message
            )
        if not v2_response.success:
            raise RuntimeError(
                "Observation v2 reset barrier failed: " + v2_response.message
            )
        v2_generation, v2_barrier = self._parse_generation_barrier(
            v2_response.message
        )
        c_generation, c_barrier = self._parse_generation_barrier(
            c_response.message
        )
        if v2_generation != c_generation or v2_barrier < c_barrier - 1.0e-9:
            raise RuntimeError(
                "Observation reset generation/order is inconsistent"
            )
        return v2_generation, v2_barrier, c_barrier

    def _reset_once(self, index):
        phase = "reset_{:02d}".format(index)
        self._set_phase(phase)
        start = self._phase_snapshot()
        wall_start = time.monotonic()
        result = {"index": index, "success": False, "failure": ""}
        try:
            with self._lock:
                pre_reset_adapted_stamp = self._last_odom_stamp.to_sec()
                pre_reset_raw_stamp = self._last_raw_stamp.to_sec()
                pre_reset_c_valid_count = sum(
                    1 for sample in self._observation_c_samples
                    if sample["valid"]
                )
            self._cancel_pub.publish(EmptyMessage())
            hold_response = self._adapter_hold()
            if not hold_response.success:
                raise RuntimeError("adapter hold failed: " + hold_response.message)
            self._wait_low_speed(4.0)

            prepare_response = self._adapter_prepare_reset()
            if not prepare_response.success:
                raise RuntimeError(
                    "adapter prepare reset failed: " + prepare_response.message
                )
            self._switch([], ["controller/pose", "controller/twist"])
            self._pause_physics()
            observation_generation = None
            observation_barrier = None
            observation_c_barrier = None
            if self._require_training_observation_c:
                (
                    observation_generation,
                    observation_barrier,
                    observation_c_barrier,
                ) = self._clear_observation_temporal_history()
            self._set_reset_model_state()
            self._unpause_physics()
            self._switch(["controller/pose", "controller/twist"], [])
            resume_response = self._adapter_resume_reset()
            if not resume_response.success:
                raise RuntimeError(
                    "adapter resume reset failed: " + resume_response.message
                )
            engage_response = self._adapter_engage()
            if not engage_response.success:
                raise RuntimeError(
                    "adapter engage failed: " + engage_response.message
                )

            reset_stamp = self._last_odom_stamp.to_sec()

            def reset_ready():
                state = self._actual_state()
                if state is None or not self._backend_ready():
                    return False
                position, velocity, roll, pitch, stamp = state
                position_error = _norm3(
                    position[0] - self._reset_pose[0],
                    position[1] - self._reset_pose[1],
                    position[2] - self._reset_pose[2],
                )
                return (
                    stamp > reset_stamp
                    and position_error <= self._thresholds["reset_max_position_error"]
                    and _norm3(*velocity) <= self._thresholds["reset_max_speed"]
                    and max(abs(roll), abs(pitch))
                    <= self._thresholds["reset_max_roll_pitch"]
                )

            if not self._wait_wall(
                reset_ready,
                max(10.0, self._reset_ready_timeout * 3.0),
                "reset readiness {}".format(index),
            ):
                raise RuntimeError("reset readiness timeout")
            self._sleep_sim(1.0, 5.0)
            first_v2 = None
            c_valid_during_reset = 0
            if self._require_training_observation_c:
                def v2_refilled():
                    with self._lock:
                        candidates = [
                            sample for sample in self._v2_samples
                            if sample["valid"]
                            and sample["generation"] == observation_generation
                            and sample["stamp"] > observation_barrier + 1.0e-9
                        ]
                        return bool(candidates)

                if not self._wait_wall(
                    v2_refilled, 10.0,
                    "Observation v2 refill after reset {}".format(index),
                ):
                    raise RuntimeError("Observation v2 did not refill after reset")
                with self._lock:
                    first_v2 = next(
                        sample for sample in self._v2_samples
                        if sample["valid"]
                        and sample["generation"] == observation_generation
                        and sample["stamp"] > observation_barrier + 1.0e-9
                    )
                    c_valid_during_reset = sum(
                        1 for sample in self._observation_c_samples
                        if sample["valid"]
                    ) - pre_reset_c_valid_count
            state = self._actual_state()
            position, velocity, roll, pitch, _ = state
            with self._lock:
                pair = dict(self._last_pair_row or {})
            result.update(
                success=True,
                ready_wall_sec=time.monotonic() - wall_start,
                final_position_error=_norm3(
                    position[0] - self._reset_pose[0],
                    position[1] - self._reset_pose[1],
                    position[2] - self._reset_pose[2],
                ),
                final_speed=_norm3(*velocity),
                final_roll_pitch=max(abs(roll), abs(pitch)),
                controller_generation=self._backend.get(
                    "controller_generation"
                ),
                pose_controller_state=self._backend.get(
                    "pose_controller_state"
                ),
                twist_controller_state=self._backend.get(
                    "twist_controller_state"
                ),
                pre_reset_raw_stamp_sec=pre_reset_raw_stamp,
                pre_reset_adapted_stamp_sec=pre_reset_adapted_stamp,
                post_reset_source_stamp_sec=pair.get("sim_time"),
                post_reset_stamp_advanced=bool(
                    pair
                    and float(pair["sim_time"])
                    > max(pre_reset_raw_stamp, pre_reset_adapted_stamp)
                ),
                post_reset_raw_adapted_stamp_equal=bool(
                    pair and pair.get("truth_stamp_equal", False)
                ),
                post_reset_position_difference_m=pair.get(
                    "truth_position_difference_m"
                ),
                post_reset_velocity_difference_mps=pair.get(
                    "truth_velocity_difference_mps"
                ),
                observation_generation=observation_generation,
                observation_barrier_stamp_sec=observation_barrier,
                observation_c_barrier_stamp_sec=observation_c_barrier,
                first_refilled_v2_stamp_sec=(
                    None if first_v2 is None else first_v2["stamp"]
                ),
                first_refilled_v2_pose_stamp_sec=(
                    None if first_v2 is None else first_v2["pose_stamp"]
                ),
                first_refilled_v2_history_frames=(
                    None if first_v2 is None else first_v2["history_frames"]
                ),
                first_refilled_v2_generation=(
                    None if first_v2 is None else first_v2["generation"]
                ),
                first_refilled_v2_after_barrier=bool(
                    first_v2 is not None
                    and first_v2["stamp"] > observation_barrier + 1.0e-9
                    and first_v2["pose_stamp"] > observation_barrier + 1.0e-9
                ) if self._require_training_observation_c else None,
                observation_c_valid_before_new_trajectory_count=(
                    c_valid_during_reset
                    if self._require_training_observation_c else None
                ),
            )
        except Exception as error:  # preserved as a qualification failure
            result["failure"] = str(error)
            self._event("reset_failure", index=index, error=str(error))
            try:
                self._unpause_physics()
            except Exception:
                pass
        summary = self._finish_phase(phase, start)
        result["phase_summary"] = summary
        self._reset_results.append(result)
        return result["success"]

    @staticmethod
    def _source_rate(samples):
        stamps = sorted(
            {float(sample["stamp"]) for sample in samples if sample["stamp"] > 0.0}
        )
        if len(stamps) < 2 or stamps[-1] <= stamps[0]:
            return None
        return (len(stamps) - 1) / (stamps[-1] - stamps[0])

    def _observation_summary(self):
        with self._lock:
            raw = list(self._raw_mid360_samples)
            v2 = list(self._v2_samples)
            observation_c = list(self._observation_c_samples)
            v2_diagnostics = dict(self._v2_diagnostics)
            c_diagnostics = dict(self._observation_c_diagnostics)
            variation = {
                phase: list(values)
                for phase, values in self._surrogate_variation_by_phase.items()
            }
        valid_v2 = [sample for sample in v2 if sample["valid"]]
        active_phases = {
            "straight", "turning", "continuous_replan", "post_reset_tracking"
        }
        active_c = [
            sample for sample in observation_c
            if sample["phase"] in active_phases
        ]
        valid_active_c = [sample for sample in active_c if sample["valid"]]
        valid_c = [sample for sample in observation_c if sample["valid"]]

        failures = []
        if not raw:
            failures.append("raw_mid360_missing")
        if not valid_v2:
            failures.append("observation_v2_missing")
        if not valid_active_c:
            failures.append("observation_c_active_missing")
        if raw and {sample["frame"] for sample in raw} != {"mid360_link"}:
            failures.append("raw_mid360_frame")
        if valid_v2 and any(
            sample["bins"] != 3200 or not sample["finite"]
            for sample in valid_v2
        ):
            failures.append("surrogate_dimension_or_finite")
        if valid_v2 and any(sample["all_zero"] for sample in valid_v2):
            failures.append("surrogate_all_zero")
        if valid_v2 and all(sample["unknown_bins"] == 3200 for sample in valid_v2):
            failures.append("surrogate_all_unknown")
        if valid_v2 and all(sample["obstacle_bins"] == 0 for sample in valid_v2):
            failures.append("surrogate_never_observes_obstacle")
        if valid_v2 and any(
            sample["pose_used_future"]
            or sample["pose_stamp"] > sample["stamp"] + 1.0e-9
            for sample in valid_v2
        ):
            failures.append("v2_future_pose_leak")
        if valid_v2 and any(
            sample["state_source"] != "gazebo_truth_training"
            or sample["state_contract"] != CONTRACT_VERSION
            or sample["cloud_frame_mode"] != "sensor_raw"
            for sample in valid_v2
        ):
            failures.append("v2_training_provenance")
        if valid_active_c and any(
            sample["policy_dimension"] != 3267
            or sample["lidar_bins"] != 3200
            or sample["future_points"] != 20
            or not sample["finite"]
            for sample in valid_active_c
        ):
            failures.append("observation_c_contract")
        if valid_active_c and any(
            sample["state_source"] != "gazebo_truth_training"
            or sample["state_contract"] != CONTRACT_VERSION
            or sample["generation"] != sample["lidar_generation"]
            or sample["lidar_pose_used_future"]
            or sample["trajectory_start"] > sample["stamp"] + 1.0e-9
            for sample in valid_active_c
        ):
            failures.append("observation_c_causality_or_provenance")
        for phase in ("straight", "turning", "continuous_replan"):
            values = variation.get(phase, [])
            if not values or max(values) <= 1.0e-5:
                failures.append("surrogate_fixed_" + phase)

        valid_rate = (
            len(valid_v2) / float(len(v2)) if v2 else 0.0
        )
        active_valid_rate = (
            len(valid_active_c) / float(len(active_c)) if active_c else 0.0
        )
        if valid_rate < 0.70:
            failures.append("observation_v2_valid_rate")
        if active_valid_rate < 0.80:
            failures.append("observation_c_active_valid_rate")

        def numeric(values, key):
            result = [float(sample[key]) for sample in values]
            return {
                "min": min(result) if result else None,
                "median": statistics.median(result) if result else None,
                "p95": _finite_or_none(_percentile(result, 0.95)),
                "max": max(result) if result else None,
            }

        variation_summary = {}
        for phase, values in variation.items():
            variation_summary[phase] = {
                "sample_count": len(values),
                "mean_l1_change": statistics.mean(values) if values else None,
                "p95_l1_change": _finite_or_none(_percentile(values, 0.95)),
                "max_l1_change": max(values) if values else None,
            }
        return {
            "raw_mid360": {
                "topic": "/uav1/livox/lidar",
                "message_type": "sensor_msgs/PointCloud2",
                "frames": sorted({sample["frame"] for sample in raw}),
                "sample_count": len(raw),
                "source_rate_hz": self._source_rate(raw),
                "points": numeric(raw, "points"),
            },
            "observation_v2": {
                "output_count": len(v2),
                "valid_count": len(valid_v2),
                "valid_ratio": valid_rate,
                "output_source_rate_hz": self._source_rate(v2),
                "accepted_source_rate_hz": self._source_rate(valid_v2),
                "finite_ratio": (
                    sum(1 for sample in valid_v2 if sample["finite"])
                    / float(len(valid_v2)) if valid_v2 else 0.0
                ),
                "unknown_bins": numeric(valid_v2, "unknown_bins"),
                "free_bins": numeric(valid_v2, "free_bins"),
                "obstacle_bins": numeric(valid_v2, "obstacle_bins"),
                "generations": sorted({sample["generation"] for sample in valid_v2}),
                "pose_lookup_modes": sorted(
                    {
                        sample.get("kinematic_lookup", "")
                        for sample in valid_active_c
                    }
                ),
                "surrogate_variation": variation_summary,
                "latest_diagnostics": v2_diagnostics,
            },
            "observation_c": {
                "output_count": len(observation_c),
                "valid_count": len(valid_c),
                "active_output_count": len(active_c),
                "active_valid_count": len(valid_active_c),
                "active_valid_ratio": active_valid_rate,
                "active_output_source_rate_hz": self._source_rate(active_c),
                "active_valid_source_rate_hz": self._source_rate(valid_active_c),
                "policy_dimensions": sorted(
                    {sample["policy_dimension"] for sample in valid_active_c}
                ),
                "actual_speed_mps": numeric(valid_active_c, "actual_speed"),
                "tracking_error_m": numeric(valid_active_c, "tracking_error"),
                "trajectory_ids": sorted(
                    {sample["trajectory_id"] for sample in valid_active_c}
                ),
                "state_sources": sorted(
                    {sample["state_source"] for sample in valid_active_c}
                ),
                "latest_diagnostics": c_diagnostics,
            },
            "failures": sorted(set(failures)),
            "pass": not failures,
        }

    def _criteria(self, truth_contract, isolation, observation_summary=None):
        failures = []
        hover = self._phase_summaries.get("hover", {})
        if (
            hover.get("position_error_max") is None
            or hover["position_error_max"]
            > self._thresholds["hover_max_position_error"]
        ):
            failures.append("hover_position_error")
        if (
            hover.get("actual_speed_max") is None
            or hover["actual_speed_max"] > self._thresholds["hover_max_speed"]
        ):
            failures.append("hover_speed")

        tracking_phases = (
            "straight",
            "turning",
            "continuous_replan",
            "post_reset_tracking",
        )
        nonideal = []
        for phase in tracking_phases:
            summary = self._phase_summaries.get(phase, {})
            if (
                summary.get("position_error_p95") is None
                or summary["position_error_p95"]
                > self._thresholds["tracking_p95_position_error"]
            ):
                failures.append(phase + "_position_p95")
            if (
                summary.get("position_error_max") is None
                or summary["position_error_max"]
                > self._thresholds["tracking_max_position_error"]
            ):
                failures.append(phase + "_position_max")
            if (
                summary.get("velocity_error_p95") is None
                or summary["velocity_error_p95"]
                > self._thresholds["tracking_p95_velocity_error"]
            ):
                failures.append(phase + "_velocity_p95")
            if summary.get("position_error_rms") is not None:
                nonideal.append(summary["position_error_rms"])
        if not nonideal or max(nonideal) < self._thresholds["minimum_nonideal_error"]:
            failures.append("tracking_error_is_ideal_zero")

        replan = self._phase_summaries.get("continuous_replan", {})
        if len(replan.get("unique_bspline_ids", [])) < self._minimum_replans:
            failures.append("continuous_replan_count")
        hold = self._phase_summaries.get("hold", {})
        if (
            hold.get("tail_speed_max") is None
            or hold["tail_speed_max"] > self._thresholds["hold_max_speed"]
        ):
            failures.append("hold_speed")
        if len(self._reset_results) != self._reset_count or not all(
            result.get("success", False) for result in self._reset_results
        ):
            failures.append("reset_failures")
        for result in self._reset_results:
            if not result.get("success"):
                continue
            if result["final_position_error"] > self._thresholds[
                "reset_max_position_error"
            ]:
                failures.append("reset_position_error")
                break
            if result["final_speed"] > self._thresholds["reset_max_speed"]:
                failures.append("reset_speed")
                break
            if result["final_roll_pitch"] > self._thresholds[
                "reset_max_roll_pitch"
            ]:
                failures.append("reset_attitude")
                break
            if (
                not result.get("post_reset_stamp_advanced", False)
                or not result.get("post_reset_raw_adapted_stamp_equal", False)
                or result.get("post_reset_position_difference_m") is None
                or result["post_reset_position_difference_m"]
                > self._thresholds["truth_max_position_difference_m"]
                or result.get("post_reset_velocity_difference_mps") is None
                or result["post_reset_velocity_difference_mps"]
                > self._thresholds["truth_max_velocity_difference_mps"]
            ):
                failures.append("reset_truth_freshness")
                break
        if truth_contract["pair_ratio"] < self._thresholds[
            "minimum_truth_pair_ratio"
        ]:
            failures.append("truth_pair_ratio")
        if not truth_contract["all_stamps_equal"]:
            failures.append("truth_stamp_mismatch")
        if truth_contract["raw_frames"] != [self._frame]:
            failures.append("raw_truth_frame")
        if truth_contract["raw_child_frames"] != [self._body_frame]:
            failures.append("raw_truth_child_frame")
        if truth_contract["adapted_frames"] != [self._frame]:
            failures.append("adapted_odom_frame")
        if truth_contract["adapted_child_frames"] != [self._body_frame]:
            failures.append("adapted_odom_child_frame")
        metric_limits = (
            ("position_difference_max_m", "truth_max_position_difference_m"),
            ("velocity_difference_max_mps", "truth_max_velocity_difference_mps"),
            ("yaw_difference_max_rad", "truth_max_yaw_difference_rad"),
            ("relay_latency_ros_p95_sec", "truth_relay_latency_p95_sec"),
        )
        for metric, threshold in metric_limits:
            if (
                truth_contract.get(metric) is None
                or truth_contract[metric] > self._thresholds[threshold]
            ):
                failures.append(metric)
        if not isolation["pass"]:
            failures.append("backend_isolation")
        truth_state = truth_contract.get("adapter_state", {})
        if (
            truth_state.get("contract_version") != CONTRACT_VERSION
            or truth_state.get("source_type") != SOURCE_TYPE
            or truth_state.get("fast_lio_provenance") is not False
            or not truth_state.get("ready", False)
            or truth_state.get("rejected_count", 0) != 0
            or truth_state.get("out_of_order_count", 0) != 0
            or truth_state.get("duplicate_count", 0) != 0
        ):
            failures.append("truth_adapter_state")
        if self._require_training_observation_c:
            if observation_summary is None or not observation_summary.get("pass"):
                failures.extend(
                    [] if observation_summary is None
                    else observation_summary.get("failures", [])
                )
                if observation_summary is None:
                    failures.append("training_observation_summary_missing")
            for result in self._reset_results:
                if not result.get("success"):
                    continue
                if (
                    not result.get("first_refilled_v2_after_barrier", False)
                    or result.get("first_refilled_v2_history_frames", 0) < 5
                    or result.get("observation_c_valid_before_new_trajectory_count") != 0
                ):
                    failures.append("observation_reset_contamination")
                    break
        return sorted(set(failures))

    def run(self):
        self._service_wait()
        if not self._wait_wall(self._backend_ready, 30.0, "backend startup readiness"):
            raise RuntimeError("backend never became ready")
        engage = self._adapter_engage()
        if not engage.success:
            raise RuntimeError("explicit engage failed: " + engage.message)

        self._run_timed_phase("hover", self._hover_duration)
        self._run_timed_phase("straight", 9.0, (2.0, 0.0, 1.5))

        self._set_phase("turning")
        turning_start = self._phase_snapshot()
        if not self._publish_goal(2.0, 2.0, 1.5):
            raise RuntimeError("turning goal publication failed")
        self._sleep_sim(3.0, 12.0)
        if not self._publish_goal(0.5, 2.0, 1.5):
            raise RuntimeError("turning re-goal publication failed")
        self._sleep_sim(8.0, 24.0)
        self._finish_phase("turning", turning_start)

        self._run_timed_phase(
            "continuous_replan", 11.0, (-2.0, -1.0, 1.5)
        )

        self._set_phase("hold")
        hold_start = self._phase_snapshot()
        if not self._publish_goal(2.0, -1.0, 1.5):
            raise RuntimeError("hold approach goal publication failed")
        self._sleep_sim(2.5, 10.0)
        self._cancel_pub.publish(EmptyMessage())
        hold = self._adapter_hold()
        if not hold.success:
            raise RuntimeError("hold service failed: " + hold.message)
        self._sleep_sim(5.0, 20.0)
        self._finish_phase("hold", hold_start)

        for index in range(1, self._reset_count + 1):
            if not self._reset_once(index):
                break

        self._run_timed_phase(
            "post_reset_tracking", 9.0, (1.5, 0.0, 1.5)
        )
        self._cancel_pub.publish(EmptyMessage())
        self._adapter_hold()
        self._sleep_sim(2.0, 8.0)

        truth_contract = self._truth_contract_summary()
        isolation = self._backend_isolation_audit()
        observation_summary = (
            self._observation_summary()
            if self._require_training_observation_c else None
        )
        failures = self._criteria(
            truth_contract, isolation, observation_summary
        )
        overall_wall = time.monotonic() - self._overall_wall_start
        overall_sim = rospy.Time.now().to_sec() - self._overall_sim_start
        summary = {
            "version": (
                "hector_training_observation_c_integration_v1.0"
                if self._require_training_observation_c
                else "hector_training_truth_odom_integration_v1.0"
            ),
            "qualification_only": True,
            "default_hector_pid_parameters_unchanged": True,
            "ego_core_modified_by_qualification": False,
            "reset_count_requested": self._reset_count,
            "reset_count_completed": len(self._reset_results),
            "phase_summaries": self._phase_summaries,
            "reset_results": self._reset_results,
            "thresholds": self._thresholds,
            "minimum_replans": self._minimum_replans,
            "truth_odometry_contract": truth_contract,
            "backend_isolation": isolation,
            "training_observation_c": observation_summary,
            "events": self._events,
            "overall": {
                "sim_duration": overall_sim,
                "wall_duration": overall_wall,
                "rtf": overall_sim / max(overall_wall, 1.0e-9),
                "counts": self._counts,
            },
            "failures": failures,
            "verdict": (
                (
                    "GO FOR TRAINING EPISODE RESET INTEGRATION"
                    if self._require_training_observation_c
                    else "GO FOR TRAINING OBSERVATION C INTEGRATION"
                )
                if not failures
                else "NO-GO"
            ),
        }
        with open(
            os.path.join(self._output_dir, "qualification_summary.json"),
            "w",
            encoding="utf-8",
        ) as stream:
            json.dump(summary, stream, indent=2, sort_keys=True, allow_nan=False)
            stream.write("\n")
        if self._require_training_observation_c:
            for filename, rows in (
                ("raw_mid360_samples.csv", self._raw_mid360_samples),
                ("observation_v2_samples.csv", self._v2_samples),
                ("observation_c_samples.csv", self._observation_c_samples),
            ):
                if not rows:
                    continue
                with open(
                    os.path.join(self._output_dir, filename), "w",
                    newline="", encoding="utf-8",
                ) as stream:
                    writer = csv.DictWriter(
                        stream, fieldnames=sorted(rows[0].keys())
                    )
                    writer.writeheader()
                    writer.writerows(rows)
        with open(
            os.path.join(self._output_dir, "qualification_events.jsonl"),
            "w",
            encoding="utf-8",
        ) as stream:
            for event in self._events:
                stream.write(json.dumps(event, sort_keys=True) + "\n")
        with self._lock:
            self._recording_open = False
            self._csv_file.flush()
            self._csv_file.close()
        rospy.logwarn("[HECTOR QUALIFICATION] verdict=%s failures=%s", summary["verdict"], failures)
        return summary


def main():
    rospy.init_node("hector_backend_qualifier")
    qualifier = HectorBackendQualifier()
    try:
        summary = qualifier.run()
    except Exception as error:
        rospy.logfatal("Hector qualification aborted: %s", error)
        raise
    if summary["verdict"] == "NO-GO":
        rospy.signal_shutdown("qualification completed with NO-GO")
    else:
        rospy.signal_shutdown("qualification completed")


if __name__ == "__main__":
    main()
