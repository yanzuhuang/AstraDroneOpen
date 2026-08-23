#!/usr/bin/env python3
"""Thin, training-only PositionCommand to Hector Pose/Twist adapter."""

import json
import math
import threading

import rospy
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PositionCommand
from std_msgs.msg import Bool, Empty as EmptyMessage, Float64, String
from std_srvs.srv import Empty, Trigger, TriggerResponse

from hector_ego_training_backend.adapter_contract import (
    CommandSample,
    StateSample,
    command_is_valid,
    quaternion_from_yaw,
    trajectory_id_is_new,
    tracking_errors,
)


def _topic(name, default):
    return rospy.get_param("~topics/" + name, default)


def _finite(values):
    return all(math.isfinite(float(value)) for value in values)


def _yaw_from_quaternion(quaternion):
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


class PositionCommandToHector:
    def __init__(self):
        self._lock = threading.RLock()
        self._enable_control = bool(rospy.get_param("~enable_control", False))
        self._qualification_only = bool(
            rospy.get_param("~qualification_only", True)
        )
        self._expected_frame = rospy.get_param("~expected_frame", "world")
        self._publish_rate = float(rospy.get_param("~publish_rate", 100.0))
        self._diagnostics_rate = float(
            rospy.get_param("~diagnostics_rate", 5.0)
        )
        self._command_freshness = float(
            rospy.get_param("~command_freshness", 0.15)
        )
        self._odometry_freshness = float(
            rospy.get_param("~odometry_freshness", 0.20)
        )
        self._future_tolerance = float(
            rospy.get_param("~future_tolerance", 0.02)
        )
        self._hover_ready_position_tolerance = float(
            rospy.get_param("~hover_ready_position_tolerance", 0.25)
        )
        self._hover_ready_velocity_tolerance = float(
            rospy.get_param("~hover_ready_velocity_tolerance", 0.15)
        )
        self._hover_ready_duration = float(
            rospy.get_param("~hover_ready_duration", 1.0)
        )
        self._auto_engage = bool(rospy.get_param("~auto_engage", True))
        self._coordinator_managed_goals = bool(
            rospy.get_param("~coordinator_managed_goals", False)
        )
        self._reset_hover = (
            float(rospy.get_param("~reset_hover_x", 0.0)),
            float(rospy.get_param("~reset_hover_y", 0.0)),
            float(rospy.get_param("~reset_hover_z", 1.5)),
            float(rospy.get_param("~reset_hover_yaw", 0.0)),
        )
        self._reset_hover_update_count = 0
        if (
            not self._expected_frame
            or not _finite(self._reset_hover)
            or self._publish_rate <= 0.0
            or self._diagnostics_rate <= 0.0
            or self._command_freshness <= 0.0
            or self._odometry_freshness <= 0.0
            or self._future_tolerance < 0.0
            or self._hover_ready_position_tolerance <= 0.0
            or self._hover_ready_velocity_tolerance <= 0.0
            or self._hover_ready_duration < 0.0
        ):
            raise ValueError("invalid adapter configuration")

        self._pose_controller = rospy.get_param(
            "~pose_controller", "controller/pose"
        )
        self._twist_controller = rospy.get_param(
            "~twist_controller", "controller/twist"
        )
        self._controller_states = {}
        self._controller_generation = 0
        self._engaged_generation = -1
        self._engage_attempts = 0
        self._engage_successes = 0

        self._mode = "DISABLED" if not self._enable_control else "WAIT_ODOMETRY"
        self._ready = False
        self._output_enabled = self._enable_control
        self._accept_commands = False
        self._goal_stamp = rospy.Time(0)
        self._trajectory_gate_armed = not self._coordinator_managed_goals
        self._cancelled_trajectory_id = 0
        self._required_trajectory_id_gt = 0
        self._last_rejection = ""
        self._last_command = None
        self._last_command_receipt = rospy.Time(0)
        self._last_command_sample = None
        self._actual_odom = None
        self._actual_receipt = rospy.Time(0)
        self._hold_position = None
        self._hold_yaw = 0.0
        self._publish_count = 0
        self._accepted_count = 0
        self._rejected_count = 0
        self._stale_hold_count = 0
        self._acceleration_consumed = False
        self._hover_ready_since = rospy.Time(0)

        self._pose_pub = rospy.Publisher(
            _topic("pose_command", "/command/pose"),
            PoseStamped,
            queue_size=1,
        )
        self._twist_pub = rospy.Publisher(
            _topic("twist_command", "/command/twist"),
            TwistStamped,
            queue_size=1,
        )
        self._acceleration_pub = rospy.Publisher(
            _topic("desired_acceleration", "desired_acceleration"),
            Vector3Stamped,
            queue_size=1,
        )
        self._position_error_pub = rospy.Publisher(
            _topic("position_tracking_error", "position_tracking_error"),
            Float64,
            queue_size=10,
        )
        self._velocity_error_pub = rospy.Publisher(
            _topic("velocity_tracking_error", "velocity_tracking_error"),
            Float64,
            queue_size=10,
        )
        self._state_pub = rospy.Publisher(
            _topic("backend_state", "backend_state"),
            String,
            queue_size=1,
            latch=True,
        )
        self._diagnostics_pub = rospy.Publisher(
            _topic("diagnostics", "diagnostics"),
            DiagnosticArray,
            queue_size=1,
        )
        self._acceleration_consumed_pub = rospy.Publisher(
            _topic("acceleration_consumed", "acceleration_consumed"),
            Bool,
            queue_size=1,
            latch=True,
        )
        self._cancel_pub = rospy.Publisher(
            _topic("cancel", "planning/cancel"),
            EmptyMessage,
            queue_size=1,
        )

        self._command_sub = rospy.Subscriber(
            _topic("position_command", "planning/pos_cmd"),
            PositionCommand,
            self._command_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self._odom_sub = rospy.Subscriber(
            _topic("odometry", "/uav1/Odometry"),
            Odometry,
            self._odometry_callback,
            queue_size=20,
            tcp_nodelay=True,
        )
        self._cancel_sub = rospy.Subscriber(
            _topic("cancel", "planning/cancel"),
            EmptyMessage,
            self._cancel_callback,
            queue_size=1,
        )
        self._goal_sub = rospy.Subscriber(
            _topic("goal", "planning/goal"),
            PoseStamped,
            self._goal_callback,
            queue_size=1,
        )
        self._reset_hover_sub = rospy.Subscriber(
            _topic("reset_hover", "training/reset_hover"),
            PoseStamped,
            self._reset_hover_callback,
            queue_size=1,
        )

        self._list_controllers = rospy.ServiceProxy(
            rospy.get_param(
                "~services/list_controllers",
                "/controller_manager/list_controllers",
            ),
            ListControllers,
            persistent=True,
        )
        self._hector_engage = rospy.ServiceProxy(
            rospy.get_param("~services/hector_engage", "/engage"),
            Empty,
            persistent=True,
        )
        self._hold_service = rospy.Service(
            "~hold", Trigger, self._hold_service_callback
        )
        self._prepare_reset_service = rospy.Service(
            "~prepare_reset", Trigger, self._prepare_reset_callback
        )
        self._resume_reset_service = rospy.Service(
            "~resume_reset_hover", Trigger, self._resume_reset_callback
        )
        self._engage_service = rospy.Service(
            "~engage", Trigger, self._engage_service_callback
        )
        self._activate_trajectory_service = rospy.Service(
            "~activate_trajectory", Trigger,
            self._activate_trajectory_service_callback,
        )

        self._acceleration_consumed_pub.publish(Bool(data=False))
        self._command_timer = rospy.Timer(
            rospy.Duration(1.0 / self._publish_rate), self._command_timer_callback
        )
        self._diagnostics_timer = rospy.Timer(
            rospy.Duration(1.0 / self._diagnostics_rate),
            self._diagnostics_timer_callback,
        )
        rospy.logwarn(
            "Hector training-only adapter active: enable_control=%s qualification_only=%s; "
            "PositionCommand acceleration is diagnostic-only and is not consumed",
            self._enable_control,
            self._qualification_only,
        )

    def _command_sample(self, message):
        return CommandSample(
            stamp_sec=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id,
            trajectory_id=int(message.trajectory_id),
            trajectory_ready=(
                message.trajectory_flag
                == PositionCommand.TRAJECTORY_STATUS_READY
            ),
            position=(
                message.position.x,
                message.position.y,
                message.position.z,
            ),
            velocity=(
                message.velocity.x,
                message.velocity.y,
                message.velocity.z,
            ),
            acceleration=(
                message.acceleration.x,
                message.acceleration.y,
                message.acceleration.z,
            ),
            yaw=message.yaw,
            yaw_dot=message.yaw_dot,
        )

    def _odometry_callback(self, message):
        position = message.pose.pose.position
        velocity = message.twist.twist.linear
        orientation = message.pose.pose.orientation
        values = (
            position.x,
            position.y,
            position.z,
            velocity.x,
            velocity.y,
            velocity.z,
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        now = rospy.Time.now()
        if (
            message.header.stamp.is_zero()
            or message.header.frame_id != self._expected_frame
            or not _finite(values)
        ):
            with self._lock:
                self._last_rejection = "invalid_odometry"
            return
        with self._lock:
            self._actual_odom = message
            self._actual_receipt = now
            if self._hold_position is None:
                self._latch_hold_locked(message)
                if self._enable_control:
                    self._mode = "STARTUP_HOLD"

    def _goal_callback(self, message):
        now = rospy.Time.now()
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        )
        if (
            message.header.stamp.is_zero()
            or message.header.frame_id != self._expected_frame
            or not _finite(values)
            or (now - message.header.stamp).to_sec() > 1.0
            or (message.header.stamp - now).to_sec() > self._future_tolerance
        ):
            with self._lock:
                self._last_rejection = "invalid_goal"
            return
        with self._lock:
            previous_trajectory_id = (
                self._last_command_sample.trajectory_id
                if self._last_command_sample is not None
                else self._cancelled_trajectory_id
            )
            self._required_trajectory_id_gt = max(
                self._required_trajectory_id_gt, previous_trajectory_id
            )
            if self._actual_odom is not None:
                # Hold the current vehicle state while EGO replaces the old
                # generation. Never jump back to the startup/reset hover.
                self._latch_hold_locked(self._actual_odom)
            self._goal_stamp = message.header.stamp
            self._trajectory_gate_armed = not self._coordinator_managed_goals
            self._accept_commands = self._trajectory_gate_armed
            self._last_command = None
            self._last_command_sample = None
            self._last_rejection = ""
            if self._mode not in ("RESET_PAUSED", "DISABLED"):
                self._mode = (
                    "WAIT_NEW_TRAJECTORY"
                    if self._accept_commands
                    else "WAIT_EPISODE_ACTIVATION"
                )

    def _cancel_callback(self, _message):
        with self._lock:
            if self._last_command_sample is not None:
                self._cancelled_trajectory_id = max(
                    self._cancelled_trajectory_id,
                    self._last_command_sample.trajectory_id,
                )
            self._accept_commands = False
            self._trajectory_gate_armed = False
            self._goal_stamp = rospy.Time(0)
            self._last_command = None
            self._last_command_sample = None
            self._required_trajectory_id_gt = max(
                self._required_trajectory_id_gt,
                self._cancelled_trajectory_id,
            )
            if self._actual_odom is not None:
                self._latch_hold_locked(self._actual_odom)
            if self._mode != "RESET_PAUSED":
                self._mode = "CANCEL_HOLD"

    def _command_callback(self, message):
        now = rospy.Time.now()
        sample = self._command_sample(message)
        valid, reason = command_is_valid(
            sample,
            self._expected_frame,
            now.to_sec(),
            self._command_freshness,
            self._future_tolerance,
        )
        with self._lock:
            if valid and not self._accept_commands:
                valid, reason = False, "trajectory_gate_closed"
            if valid and sample.stamp_sec + 1.0e-9 < self._goal_stamp.to_sec():
                valid, reason = False, "command_precedes_goal"
            if (
                valid
                and not trajectory_id_is_new(
                    sample.trajectory_id, self._required_trajectory_id_gt
                )
            ):
                valid, reason = False, "old_trajectory_generation"
            if not valid:
                self._rejected_count += 1
                self._last_rejection = reason
                return
            self._last_command = message
            self._last_command_sample = sample
            self._last_command_receipt = now
            self._accepted_count += 1
            self._last_rejection = ""
            self._mode = "TRACK"

    def _latch_hold_locked(self, odometry):
        position = odometry.pose.pose.position
        self._hold_position = (position.x, position.y, position.z)
        self._hold_yaw = _yaw_from_quaternion(odometry.pose.pose.orientation)
        self._hover_ready_since = rospy.Time(0)

    def _reset_hover_callback(self, message):
        quaternion = message.pose.orientation
        values = (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )
        norm = math.sqrt(
            quaternion.x ** 2
            + quaternion.y ** 2
            + quaternion.z ** 2
            + quaternion.w ** 2
        )
        with self._lock:
            if self._mode != "RESET_PAUSED" or self._output_enabled:
                self._last_rejection = "reset_hover_outside_reset_pause"
                return
            if (
                message.header.frame_id.lstrip("/")
                != self._expected_frame.lstrip("/")
                or not _finite(values)
                or abs(norm - 1.0) > 1.0e-6
            ):
                self._last_rejection = "invalid_reset_hover_target"
                return
            roll_pitch = (
                2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z),
                2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x),
            )
            if max(abs(value) for value in roll_pitch) > 1.0e-6:
                self._last_rejection = "reset_hover_roll_pitch_nonzero"
                return
            self._reset_hover = (
                float(message.pose.position.x),
                float(message.pose.position.y),
                float(message.pose.position.z),
                _yaw_from_quaternion(quaternion),
            )
            self._reset_hover_update_count += 1
            self._last_rejection = ""

    def _configured_reset_hold_locked(self):
        self._hold_position = self._reset_hover[:3]
        self._hold_yaw = self._reset_hover[3]
        self._hover_ready_since = rospy.Time(0)

    def _reference_locked(self, now):
        if self._mode == "TRACK" and self._last_command_sample is not None:
            receipt_age = (now - self._last_command_receipt).to_sec()
            source_age = now.to_sec() - self._last_command_sample.stamp_sec
            if (
                receipt_age <= self._command_freshness
                and source_age <= self._command_freshness
                and source_age >= -self._future_tolerance
            ):
                return self._last_command_sample
            if self._actual_odom is not None:
                self._latch_hold_locked(self._actual_odom)
            self._cancelled_trajectory_id = max(
                self._cancelled_trajectory_id,
                self._last_command_sample.trajectory_id,
            )
            self._required_trajectory_id_gt = max(
                self._required_trajectory_id_gt,
                self._cancelled_trajectory_id,
            )
            self._last_command = None
            self._last_command_sample = None
            self._accept_commands = False
            self._trajectory_gate_armed = False
            self._goal_stamp = rospy.Time(0)
            self._mode = "STALE_HOLD"
            self._last_rejection = "command_stream_stale"
            self._stale_hold_count += 1
            self._cancel_pub.publish(EmptyMessage())

        if self._hold_position is None:
            return None
        return CommandSample(
            stamp_sec=now.to_sec(),
            frame_id=self._expected_frame,
            trajectory_id=max(1, self._cancelled_trajectory_id),
            trajectory_ready=True,
            position=self._hold_position,
            velocity=(0.0, 0.0, 0.0),
            acceleration=(0.0, 0.0, 0.0),
            yaw=self._hold_yaw,
            yaw_dot=0.0,
        )

    def _publish_reference(self, sample, now):
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = self._expected_frame
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = (
            sample.position
        )
        quaternion = quaternion_from_yaw(sample.yaw)
        (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ) = quaternion

        twist = TwistStamped()
        twist.header = pose.header
        (
            twist.twist.linear.x,
            twist.twist.linear.y,
            twist.twist.linear.z,
        ) = sample.velocity
        twist.twist.angular.z = sample.yaw_dot

        acceleration = Vector3Stamped()
        acceleration.header = pose.header
        (
            acceleration.vector.x,
            acceleration.vector.y,
            acceleration.vector.z,
        ) = sample.acceleration

        self._pose_pub.publish(pose)
        self._twist_pub.publish(twist)
        self._acceleration_pub.publish(acceleration)
        with self._lock:
            self._publish_count += 1
            actual = self._actual_odom
        if actual is not None:
            state = StateSample(
                position=(
                    actual.pose.pose.position.x,
                    actual.pose.pose.position.y,
                    actual.pose.pose.position.z,
                ),
                velocity=(
                    actual.twist.twist.linear.x,
                    actual.twist.twist.linear.y,
                    actual.twist.twist.linear.z,
                ),
            )
            _, _, position_norm, velocity_norm = tracking_errors(sample, state)
            self._position_error_pub.publish(Float64(data=position_norm))
            self._velocity_error_pub.publish(Float64(data=velocity_norm))

    def _command_timer_callback(self, _event):
        if not self._enable_control:
            return
        now = rospy.Time.now()
        if now.is_zero():
            return
        with self._lock:
            if not self._output_enabled:
                return
            sample = self._reference_locked(now)
        if sample is not None:
            self._publish_reference(sample, now)

    def _controllers_running_locked(self):
        return (
            self._controller_states.get(self._pose_controller) == "running"
            and self._controller_states.get(self._twist_controller) == "running"
        )

    def _refresh_controller_states(self):
        try:
            response = self._list_controllers(ListControllersRequest())
        except (rospy.ServiceException, rospy.ROSException):
            return False
        states = {controller.name: controller.state for controller in response.controller}
        with self._lock:
            was_running = self._controllers_running_locked()
            self._controller_states = states
            running = self._controllers_running_locked()
            if running and not was_running:
                self._controller_generation += 1
                self._engaged_generation = -1
        return running

    def _try_engage(self):
        with self._lock:
            if (
                not self._enable_control
                or not self._output_enabled
                or self._hold_position is None
                or not self._controllers_running_locked()
                or self._engaged_generation == self._controller_generation
            ):
                return False, "controllers/output/hover not ready"
            generation = self._controller_generation
            self._engage_attempts += 1
        try:
            self._hector_engage()
        except (rospy.ServiceException, rospy.ROSException) as error:
            return False, str(error)
        with self._lock:
            self._engaged_generation = generation
            self._engage_successes += 1
        return True, "engaged"

    def _hold_service_callback(self, _request):
        with self._lock:
            if self._actual_odom is None:
                return TriggerResponse(False, "odometry unavailable")
            self._latch_hold_locked(self._actual_odom)
            self._accept_commands = False
            self._trajectory_gate_armed = False
            self._goal_stamp = rospy.Time(0)
            self._last_command = None
            self._last_command_sample = None
            self._mode = "SERVICE_HOLD"
        self._cancel_pub.publish(EmptyMessage())
        return TriggerResponse(True, "current pose latched as hover")

    def _prepare_reset_callback(self, _request):
        with self._lock:
            self._accept_commands = False
            self._trajectory_gate_armed = False
            self._goal_stamp = rospy.Time(0)
            self._last_command = None
            self._last_command_sample = None
            self._output_enabled = False
            self._ready = False
            self._mode = "RESET_PAUSED"
            self._engaged_generation = -1
        self._cancel_pub.publish(EmptyMessage())
        return TriggerResponse(True, "adapter output paused for controller reset")

    def _resume_reset_callback(self, _request):
        if not self._enable_control:
            return TriggerResponse(False, "control disabled")
        now = rospy.Time.now()
        with self._lock:
            self._configured_reset_hold_locked()
            self._output_enabled = True
            self._accept_commands = False
            self._trajectory_gate_armed = False
            self._goal_stamp = rospy.Time(0)
            self._last_command = None
            self._last_command_sample = None
            self._mode = "RESET_HOLD"
            self._engaged_generation = -1
            sample = self._reference_locked(now)
        if sample is not None:
            self._publish_reference(sample, now)
        return TriggerResponse(True, "fresh configured hover command published")

    def _engage_service_callback(self, _request):
        running = self._refresh_controller_states()
        if not running:
            return TriggerResponse(False, "Pose/Twist controllers are not running")
        success, message = self._try_engage()
        if not success:
            with self._lock:
                already = (
                    self._engaged_generation == self._controller_generation
                )
            if already:
                return TriggerResponse(True, "already engaged for controller generation")
        return TriggerResponse(success, message)

    def _activate_trajectory_service_callback(self, _request):
        if not self._coordinator_managed_goals:
            return TriggerResponse(
                False, "coordinator_managed_goals is disabled"
            )
        with self._lock:
            if not self._enable_control or not self._output_enabled:
                return TriggerResponse(False, "adapter output is disabled")
            if self._goal_stamp.is_zero():
                return TriggerResponse(False, "no fresh goal is armed")
            if not self._controllers_running_locked():
                return TriggerResponse(False, "controllers are not running")
            if self._engaged_generation != self._controller_generation:
                return TriggerResponse(False, "controllers are not engaged")
            self._trajectory_gate_armed = True
            self._accept_commands = True
            self._last_command = None
            self._last_command_sample = None
            self._mode = "WAIT_NEW_TRAJECTORY"
            self._last_rejection = ""
        return TriggerResponse(True, "fresh episode trajectory gate activated")

    def _state_payload_locked(self, now):
        odom_age = (
            math.inf
            if self._actual_receipt.is_zero()
            else max(0.0, (now - self._actual_receipt).to_sec())
        )
        command_age = (
            math.inf
            if self._last_command_receipt.is_zero()
            else max(0.0, (now - self._last_command_receipt).to_sec())
        )
        controllers_running = self._controllers_running_locked()
        engaged = self._engaged_generation == self._controller_generation
        hover_condition = False
        hover_position_error = math.inf
        actual_speed = math.inf
        if self._actual_odom is not None and self._hold_position is not None:
            actual_position = self._actual_odom.pose.pose.position
            actual_velocity = self._actual_odom.twist.twist.linear
            hover_position_error = math.sqrt(
                (self._hold_position[0] - actual_position.x) ** 2
                + (self._hold_position[1] - actual_position.y) ** 2
                + (self._hold_position[2] - actual_position.z) ** 2
            )
            actual_speed = math.sqrt(
                actual_velocity.x ** 2
                + actual_velocity.y ** 2
                + actual_velocity.z ** 2
            )
            hover_condition = (
                hover_position_error <= self._hover_ready_position_tolerance
                and actual_speed <= self._hover_ready_velocity_tolerance
            )
        hover_mode = self._mode != "TRACK"
        if hover_mode and hover_condition:
            if self._hover_ready_since.is_zero():
                self._hover_ready_since = now
        elif hover_mode:
            self._hover_ready_since = rospy.Time(0)
        hover_sustained = (
            not hover_mode
            or (
                not self._hover_ready_since.is_zero()
                and (now - self._hover_ready_since).to_sec()
                >= self._hover_ready_duration
            )
        )
        self._ready = bool(
            self._enable_control
            and self._output_enabled
            and controllers_running
            and engaged
            and self._hold_position is not None
            and odom_age <= self._odometry_freshness
            and self._mode not in ("WAIT_ODOMETRY", "RESET_PAUSED", "DISABLED")
            and hover_sustained
        )
        return {
            "version": "hector_ego_backend_state_v1.0",
            "qualification_only": self._qualification_only,
            "mode": self._mode,
            "ready": self._ready,
            "enable_control": self._enable_control,
            "output_enabled": self._output_enabled,
            "coordinator_managed_goals": self._coordinator_managed_goals,
            "trajectory_gate_armed": self._trajectory_gate_armed,
            "controllers_running": controllers_running,
            "pose_controller_state": self._controller_states.get(
                self._pose_controller, "missing"
            ),
            "twist_controller_state": self._controller_states.get(
                self._twist_controller, "missing"
            ),
            "controller_generation": self._controller_generation,
            "engaged_generation": self._engaged_generation,
            "odometry_age_sec": odom_age,
            "command_age_sec": command_age,
            "hover_position_error": hover_position_error,
            "actual_speed": actual_speed,
            "hover_ready_sustained": hover_sustained,
            "configured_reset_hover": list(self._reset_hover),
            "reset_hover_update_count": self._reset_hover_update_count,
            "accepted_commands": self._accepted_count,
            "rejected_commands": self._rejected_count,
            "published_commands": self._publish_count,
            "stale_hold_count": self._stale_hold_count,
            "last_rejection": self._last_rejection,
            "acceleration_consumed": self._acceleration_consumed,
            "engage_attempts": self._engage_attempts,
            "engage_successes": self._engage_successes,
        }

    def _diagnostics_timer_callback(self, _event):
        self._refresh_controller_states()
        if self._auto_engage:
            self._try_engage()
        now = rospy.Time.now()
        with self._lock:
            payload = self._state_payload_locked(now)
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/execution_backend"
        status.hardware_id = "hector_quadrotor_training_only"
        status.level = (
            DiagnosticStatus.OK if payload["ready"] else DiagnosticStatus.WARN
        )
        status.message = payload["mode"]
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in sorted(payload.items())
        ]
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = now
        diagnostics.status = [status]
        self._diagnostics_pub.publish(diagnostics)


def main():
    rospy.init_node("position_command_to_hector")
    PositionCommandToHector()
    rospy.spin()


if __name__ == "__main__":
    main()
