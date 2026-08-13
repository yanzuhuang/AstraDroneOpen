#!/usr/bin/env python3
"""Observation -> policy -> safety filter -> EGO dynamic v_max adapter."""

import math
import threading

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32MultiArray, Float64

from learning_speed_rl.observation import (
    LOW_DIM_FIELDS,
    LowDimObservationBuilder,
    MapTensorSpec,
    OccupiedPointCloudVoxelizer,
    VehiclePlanningState,
)
from learning_speed_rl.policy import (
    MockSpeedPolicy,
    SafetyFilterConfig,
    SpeedSafetyFilter,
)


def _xyz(message):
    return np.asarray([message.x, message.y, message.z], dtype=np.float32)


def _finite_xyz(values):
    return values.shape == (3,) and bool(np.all(np.isfinite(values)))


class SpeedAdapterNode:
    def __init__(self):
        self._lock = threading.RLock()

        policy_mode = str(rospy.get_param("~policy/mode", "mock")).strip().lower()
        if policy_mode != "mock":
            raise rospy.ROSInitException(
                "only the reviewed mock policy is enabled in this integration; "
                "RL inference requires an explicit backend and reviewed model"
            )
        self._policy_mode = policy_mode

        safety_config = SafetyFilterConfig(
            v_max_min=float(rospy.get_param("~safety/v_max_min", 0.05)),
            v_max_max=float(rospy.get_param("~safety/v_max_max", 0.20)),
            initial_v_max=float(rospy.get_param("~safety/initial_v_max", 0.20)),
            rise_rate_mps2=float(rospy.get_param("~safety/rise_rate_mps2", 0.08)),
            fall_rate_mps2=float(rospy.get_param("~safety/fall_rate_mps2", 0.12)),
            maximum_step_mps=float(rospy.get_param("~safety/maximum_step_mps", 0.02)),
            low_pass_alpha=float(rospy.get_param("~safety/low_pass_alpha", 0.45)),
            hysteresis_mps=float(rospy.get_param("~safety/hysteresis_mps", 0.005)),
        )
        safety_config.validate()
        self._filter = SpeedSafetyFilter(safety_config)
        self._applied_tolerance = float(
            rospy.get_param("~safety/applied_tolerance_mps", 0.005)
        )
        if not math.isfinite(self._applied_tolerance) or self._applied_tolerance < 0.0:
            raise rospy.ROSInitException("safety/applied_tolerance_mps is invalid")

        mock_default = float(
            rospy.get_param("~policy/mock_default_v_max", safety_config.initial_v_max)
        )
        self._policy = MockSpeedPolicy(mock_default)

        map_size = tuple(
            float(value)
            for value in rospy.get_param(
                "~observation/map_size_xyz_m", [24.0, 24.0, 8.0]
            )
        )
        resolution = float(
            rospy.get_param("~observation/map_resolution_m", 0.50)
        )
        self._map_spec = MapTensorSpec(map_size, resolution)
        self._voxelizer = OccupiedPointCloudVoxelizer(self._map_spec)
        normalization = rospy.get_param(
            "~observation/normalization_scales", [1.0] * len(LOW_DIM_FIELDS)
        )
        self._observation_builder = LowDimObservationBuilder(normalization)
        self._maximum_cloud_points = int(
            rospy.get_param("~observation/maximum_cloud_points", 200000)
        )
        self._velocity_alpha = float(
            rospy.get_param("~observation/velocity_filter_alpha", 0.30)
        )
        self._acceleration_alpha = float(
            rospy.get_param("~observation/acceleration_filter_alpha", 0.20)
        )
        if (
            self._maximum_cloud_points <= 0
            or not 0.0 < self._velocity_alpha <= 1.0
            or not 0.0 < self._acceleration_alpha <= 1.0
        ):
            raise rospy.ROSInitException("invalid observation filter parameters")

        self._policy_rate = float(rospy.get_param("~timing/policy_rate_hz", 10.0))
        self._odom_timeout = float(rospy.get_param("~timing/odom_timeout_sec", 0.5))
        self._command_timeout = float(
            rospy.get_param("~timing/command_timeout_sec", 0.5)
        )
        self._occupancy_timeout = float(
            rospy.get_param("~timing/occupancy_timeout_sec", 1.0)
        )
        if (
            self._policy_rate <= 0.0
            or self._odom_timeout <= 0.0
            or self._command_timeout <= 0.0
            or self._occupancy_timeout <= 0.0
        ):
            raise rospy.ROSInitException("invalid policy timing parameters")

        topic = lambda name, default: rospy.get_param("~topics/" + name, default)
        self._topic_odom = topic("odom", "Odometry")
        self._topic_occupancy = topic("occupancy", "stage3/occupancy_inflate")
        self._topic_command = topic("position_command", "planning/pos_cmd")
        self._topic_goal = topic("local_goal", "move_base_simple/goal")
        self._topic_mock = topic("mock_v_max", "learning_speed/mock_v_max")
        self._topic_raw = topic("raw_v_max", "learning_speed/raw_v_max")
        self._topic_safe = topic("safe_v_max", "learning_speed/v_max")
        self._topic_applied = topic(
            "ego_applied_v_max", "learning_speed/applied_v_max"
        )
        self._topic_ready = topic(
            "observation_ready", "learning_speed/observation_ready"
        )
        self._topic_low_dim = topic(
            "low_dim_observation", "learning_speed/observation/low_dim"
        )
        self._topic_diagnostics = topic(
            "diagnostics", "learning_speed/diagnostics"
        )

        self._state = None
        self._previous_position = None
        self._previous_odom_stamp = None
        self._latest_map_tensor = None
        self._latest_map_frame = ""
        self._latest_map_receive_sec = None
        self._latest_odom_receive_sec = None
        self._latest_command_receive_sec = None
        self._occupied_voxel_count = 0
        self._map_frame_mismatch = False
        self._command_frame_mismatch = False
        self._goal_frame_mismatch = False
        self._last_applied_v_max = None
        self._last_applied_receive_sec = None
        self._previous_safe_v_max = safety_config.initial_v_max
        self._last_policy_time = None

        self._raw_pub = rospy.Publisher(self._topic_raw, Float64, queue_size=1)
        self._safe_pub = rospy.Publisher(self._topic_safe, Float64, queue_size=1)
        self._ready_pub = rospy.Publisher(
            self._topic_ready, Bool, queue_size=1, latch=True
        )
        self._low_dim_pub = rospy.Publisher(
            self._topic_low_dim, Float32MultiArray, queue_size=1
        )
        self._diagnostic_pub = rospy.Publisher(
            self._topic_diagnostics, DiagnosticArray, queue_size=1
        )

        self._odom_sub = rospy.Subscriber(
            self._topic_odom, Odometry, self._odom_callback, queue_size=1
        )
        self._occupancy_sub = rospy.Subscriber(
            self._topic_occupancy,
            PointCloud2,
            self._occupancy_callback,
            queue_size=1,
        )
        self._command_sub = rospy.Subscriber(
            self._topic_command,
            PositionCommand,
            self._command_callback,
            queue_size=1,
        )
        self._goal_sub = rospy.Subscriber(
            self._topic_goal, PoseStamped, self._goal_callback, queue_size=1
        )
        self._mock_sub = rospy.Subscriber(
            self._topic_mock, Float64, self._mock_callback, queue_size=1
        )
        self._applied_sub = rospy.Subscriber(
            self._topic_applied, Float64, self._applied_callback, queue_size=1
        )

        self._timer = rospy.Timer(
            rospy.Duration(1.0 / self._policy_rate), self._policy_timer
        )
        rospy.logwarn(
            "learning_speed_rl mock adapter active: %s -> %s; RL inference is disabled",
            rospy.resolve_name(self._topic_mock),
            rospy.resolve_name(self._topic_safe),
        )

    def _odom_callback(self, message):
        position = _xyz(message.pose.pose.position)
        twist_velocity = _xyz(message.twist.twist.linear)
        if not _finite_xyz(position) or not _finite_xyz(twist_velocity):
            rospy.logerr_throttle(1.0, "learning_speed_rl rejected non-finite odometry")
            return
        stamp = message.header.stamp.to_sec()
        if stamp <= 0.0:
            stamp = rospy.Time.now().to_sec()

        with self._lock:
            velocity = twist_velocity
            acceleration = np.zeros(3, dtype=np.float32)
            if self._state is not None:
                velocity = self._state.velocity.copy()
                acceleration = self._state.acceleration.copy()
            if self._previous_position is not None and self._previous_odom_stamp is not None:
                dt = stamp - self._previous_odom_stamp
                if 1.0e-3 <= dt <= 1.0:
                    measured_velocity = (position - self._previous_position) / dt
                    velocity = (
                        self._velocity_alpha * measured_velocity
                        + (1.0 - self._velocity_alpha) * velocity
                    ).astype(np.float32)
                    measured_acceleration = (velocity - self._state.velocity) / dt
                    acceleration = (
                        self._acceleration_alpha * measured_acceleration
                        + (1.0 - self._acceleration_alpha) * acceleration
                    ).astype(np.float32)

            if self._state is None:
                self._state = VehiclePlanningState(
                    stamp_sec=stamp,
                    frame_id=message.header.frame_id,
                )
            self._state.stamp_sec = stamp
            self._state.frame_id = message.header.frame_id
            self._state.position = position
            self._state.velocity = velocity
            self._state.acceleration = acceleration
            self._previous_position = position.copy()
            self._previous_odom_stamp = stamp
            self._latest_odom_receive_sec = rospy.Time.now().to_sec()

    def _command_callback(self, message):
        desired_position = _xyz(message.position)
        desired_velocity = _xyz(message.velocity)
        desired_acceleration = _xyz(message.acceleration)
        if not all(
            _finite_xyz(value)
            for value in (desired_position, desired_velocity, desired_acceleration)
        ):
            rospy.logerr_throttle(
                1.0, "learning_speed_rl rejected non-finite PositionCommand"
            )
            return
        with self._lock:
            if self._state is None:
                rospy.logwarn_throttle(
                    1.0, "learning_speed_rl waits for odometry before PositionCommand"
                )
                return
            if message.header.frame_id != self._state.frame_id:
                self._command_frame_mismatch = True
                rospy.logerr_throttle(
                    1.0,
                    "learning_speed_rl requires command and odom in one frame: %s != %s",
                    message.header.frame_id,
                    self._state.frame_id,
                )
                return
            self._state.desired_position = desired_position
            self._state.desired_velocity = desired_velocity
            self._state.desired_acceleration = desired_acceleration
            self._state.have_command = True
            self._latest_command_receive_sec = rospy.Time.now().to_sec()
            self._command_frame_mismatch = False

    def _goal_callback(self, message):
        goal = _xyz(message.pose.position)
        if not _finite_xyz(goal):
            return
        with self._lock:
            if self._state is None:
                rospy.logwarn_throttle(
                    1.0, "learning_speed_rl waits for odometry before local goal"
                )
                return
            if message.header.frame_id != self._state.frame_id:
                self._goal_frame_mismatch = True
                rospy.logerr_throttle(
                    1.0,
                    "learning_speed_rl requires goal and odom in one frame: %s != %s",
                    message.header.frame_id,
                    self._state.frame_id,
                )
                return
            self._state.local_goal = goal
            self._state.have_goal = True
            self._goal_frame_mismatch = False

    def _occupancy_callback(self, message):
        with self._lock:
            if self._state is None:
                return
            center = self._state.position.copy()
            odom_frame = self._state.frame_id
            trajectory_points = (
                [self._state.desired_position.copy()]
                if self._state.have_command
                else []
            )

        if odom_frame and message.header.frame_id != odom_frame:
            with self._lock:
                self._map_frame_mismatch = True
            rospy.logerr_throttle(
                1.0,
                "learning_speed_rl requires occupancy and odom in one frame: %s != %s",
                message.header.frame_id,
                odom_frame,
            )
            return

        points = []
        try:
            for index, point in enumerate(
                point_cloud2.read_points(
                    message, field_names=("x", "y", "z"), skip_nans=True
                )
            ):
                if index >= self._maximum_cloud_points:
                    rospy.logwarn_throttle(
                        2.0,
                        "learning_speed_rl occupancy crop reached maximum_cloud_points",
                    )
                    break
                points.append(point)
        except (KeyError, ValueError) as error:
            rospy.logerr_throttle(
                1.0, "learning_speed_rl could not decode occupancy cloud: %s", error
            )
            return

        tensor = self._voxelizer.build(points, center, trajectory_points)
        with self._lock:
            self._latest_map_tensor = tensor
            self._latest_map_frame = message.header.frame_id
            self._latest_map_receive_sec = rospy.Time.now().to_sec()
            self._occupied_voxel_count = int(np.count_nonzero(tensor[1]))
            self._map_frame_mismatch = False

    def _mock_callback(self, message):
        try:
            self._policy.set_command(message.data)
        except ValueError as error:
            rospy.logerr_throttle(1.0, "learning_speed_rl rejected mock command: %s", error)

    def _applied_callback(self, message):
        if not math.isfinite(message.data):
            return
        with self._lock:
            self._last_applied_v_max = float(message.data)
            self._last_applied_receive_sec = rospy.Time.now().to_sec()

    def _snapshot_observation(self, now_sec):
        with self._lock:
            if self._state is None:
                state = VehiclePlanningState(now_sec, "")
            else:
                state = VehiclePlanningState(
                    stamp_sec=self._state.stamp_sec,
                    frame_id=self._state.frame_id,
                    position=self._state.position.copy(),
                    velocity=self._state.velocity.copy(),
                    acceleration=self._state.acceleration.copy(),
                    desired_position=self._state.desired_position.copy(),
                    desired_velocity=self._state.desired_velocity.copy(),
                    desired_acceleration=self._state.desired_acceleration.copy(),
                    local_goal=self._state.local_goal.copy(),
                    have_command=self._state.have_command,
                    have_goal=self._state.have_goal,
                )
            map_tensor = (
                None
                if self._latest_map_tensor is None
                else self._latest_map_tensor.copy()
            )
            metadata = {
                "map_source": "ego_inflated_occupied_pointcloud",
                "occupied_voxel_count": self._occupied_voxel_count,
            }

        # The current point-cloud map does not encode observed free space, and
        # one instantaneous PositionCommand is not the paper's preplanned path.
        return self._observation_builder.build(
            state=state,
            previous_v_max=self._previous_safe_v_max,
            map_spec=self._map_spec,
            map_tensor=map_tensor,
            map_semantics_complete=False,
            trajectory_context_complete=False,
            metadata=metadata,
        )

    @staticmethod
    def _age(now_sec, received_sec):
        if received_sec is None:
            return math.inf
        return max(0.0, now_sec - received_sec)

    def _policy_timer(self, _event):
        now_sec = rospy.Time.now().to_sec()
        if self._last_policy_time is not None and now_sec < self._last_policy_time:
            self._filter.reset_time()
        self._last_policy_time = now_sec

        try:
            observation = self._snapshot_observation(now_sec)
            raw_v_max = self._policy.predict(observation)
            safe_v_max = self._filter.update(raw_v_max, now_sec)
        except (RuntimeError, ValueError) as error:
            rospy.logerr_throttle(1.0, "learning_speed_rl policy cycle failed: %s", error)
            return

        self._raw_pub.publish(Float64(data=raw_v_max))
        self._safe_pub.publish(Float64(data=safe_v_max))
        self._low_dim_pub.publish(
            Float32MultiArray(data=observation.normalized_low_dim.tolist())
        )
        self._ready_pub.publish(Bool(data=observation.ready_for_rl))
        self._previous_safe_v_max = safe_v_max
        self._publish_diagnostics(now_sec, raw_v_max, safe_v_max, observation)

    def _publish_diagnostics(self, now_sec, raw_v_max, safe_v_max, observation):
        with self._lock:
            odom_age = self._age(now_sec, self._latest_odom_receive_sec)
            command_age = self._age(now_sec, self._latest_command_receive_sec)
            occupancy_age = self._age(now_sec, self._latest_map_receive_sec)
            applied_age = self._age(now_sec, self._last_applied_receive_sec)
            applied = self._last_applied_v_max
            frame_mismatch = self._map_frame_mismatch
            command_frame_mismatch = self._command_frame_mismatch
            goal_frame_mismatch = self._goal_frame_mismatch

        warnings = []
        if odom_age > self._odom_timeout:
            warnings.append("odom stale")
        if command_age > self._command_timeout:
            warnings.append("position command stale")
        if occupancy_age > self._occupancy_timeout:
            warnings.append("occupancy stale")
        if frame_mismatch:
            warnings.append("map/odom frame mismatch")
        if command_frame_mismatch:
            warnings.append("command/odom frame mismatch")
        if goal_frame_mismatch:
            warnings.append("goal/odom frame mismatch")
        if applied is None or applied_age > 1.0:
            warnings.append("EGO applied limit unavailable")
        elif abs(applied - safe_v_max) > self._applied_tolerance:
            warnings.append("EGO applied limit differs from adapter output")

        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/speed_adapter"
        status.hardware_id = "software"
        status.level = DiagnosticStatus.WARN if warnings else DiagnosticStatus.OK
        status.message = "; ".join(warnings) if warnings else "mock speed chain healthy"
        values = {
            "policy_mode": self._policy_mode,
            "raw_v_max_mps": "{:.6f}".format(raw_v_max),
            "safe_v_max_mps": "{:.6f}".format(safe_v_max),
            "ego_applied_v_max_mps": "unavailable" if applied is None else "{:.6f}".format(applied),
            "observation_ready_for_rl": str(observation.ready_for_rl).lower(),
            "map_semantics_complete": str(observation.map_semantics_complete).lower(),
            "trajectory_context_complete": str(observation.trajectory_context_complete).lower(),
            "map_tensor_shape": str(observation.map_spec.tensor_shape),
            "map_frame": observation.frame_id,
            "occupied_voxel_count": str(observation.metadata.get("occupied_voxel_count", 0)),
            "odom_age_sec": "{:.3f}".format(odom_age),
            "command_age_sec": "{:.3f}".format(command_age),
            "occupancy_age_sec": "{:.3f}".format(occupancy_age),
        }
        status.values = [KeyValue(key=key, value=value) for key, value in values.items()]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self._diagnostic_pub.publish(message)


def main():
    rospy.init_node("speed_adapter")
    SpeedAdapterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
