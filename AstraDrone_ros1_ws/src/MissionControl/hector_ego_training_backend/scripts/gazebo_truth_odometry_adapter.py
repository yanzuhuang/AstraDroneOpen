#!/usr/bin/env python3
"""Expose Gazebo p3d truth through AstraDroneOpen's training odom contract."""

import copy
import json
import math
import threading

import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String

from hector_ego_training_backend.truth_odom_contract import (
    BACKEND_MODE,
    CONTRACT_VERSION,
    SOURCE_TYPE,
    VELOCITY_FRAME,
    TruthOdometrySample,
    normalize_frame,
    validate_truth_sample,
    vector_difference_norm,
)


def _topic(name, default):
    return str(rospy.get_param("~topics/" + name, default))


class GazeboTruthOdometryAdapter:
    def __init__(self):
        self._lock = threading.RLock()
        self._backend_mode = str(
            rospy.get_param("~backend_mode", BACKEND_MODE)
        )
        if self._backend_mode != BACKEND_MODE:
            raise rospy.ROSInitException(
                "truth adapter requires backend_mode={}".format(BACKEND_MODE)
            )
        self._input_topic = rospy.resolve_name(
            _topic("raw_truth", "/ground_truth/state")
        )
        self._output_topic = rospy.resolve_name(
            _topic("odometry", "/uav1/Odometry")
        )
        if self._input_topic == self._output_topic:
            raise rospy.ROSInitException("raw truth and adapted odom must differ")

        self._expected_raw_frame = str(
            rospy.get_param("~frames/raw_world", "world")
        )
        self._expected_raw_child = str(
            rospy.get_param("~frames/raw_body", "base_link")
        )
        self._output_frame = str(
            rospy.get_param("~frames/planning_world", "world")
        )
        self._output_child = str(
            rospy.get_param("~frames/planning_body", "base_link")
        )
        if normalize_frame(self._expected_raw_frame) != normalize_frame(
            self._output_frame
        ) or normalize_frame(self._expected_raw_child) != normalize_frame(
            self._output_child
        ):
            raise rospy.ROSInitException(
                "this v1 adapter permits only identity-aligned world/body frames"
            )

        self._maximum_age = float(
            rospy.get_param("~timing/maximum_source_age_sec", 0.05)
        )
        self._future_tolerance = float(
            rospy.get_param("~timing/future_tolerance_sec", 0.01)
        )
        self._diagnostics_rate = float(
            rospy.get_param("~timing/diagnostics_rate_hz", 5.0)
        )
        self._quaternion_tolerance = float(
            rospy.get_param("~validation/quaternion_norm_tolerance", 1.0e-3)
        )
        self._teleport_distance = float(
            rospy.get_param("~validation/teleport_distance_m", 0.75)
        )
        if (
            self._maximum_age <= 0.0
            or self._future_tolerance < 0.0
            or self._diagnostics_rate <= 0.0
            or self._quaternion_tolerance <= 0.0
            or self._teleport_distance <= 0.0
        ):
            raise rospy.ROSInitException("invalid truth odometry configuration")

        self._assert_output_has_no_publisher()
        self._accepted = 0
        self._rejected = 0
        self._out_of_order = 0
        self._duplicate = 0
        self._teleport_discontinuities = 0
        self._last_rejection = ""
        self._last_stamp = rospy.Time(0)
        self._last_position = None
        self._last_callback_latency = math.nan

        self._odom_pub = rospy.Publisher(
            self._output_topic, Odometry, queue_size=1, tcp_nodelay=True
        )
        self._metadata_pub = rospy.Publisher(
            _topic("metadata", "training/odometry/metadata"),
            String,
            queue_size=1,
            latch=True,
        )
        self._diagnostics_pub = rospy.Publisher(
            _topic("diagnostics", "training/odometry/diagnostics"),
            DiagnosticArray,
            queue_size=1,
        )
        self._state_pub = rospy.Publisher(
            _topic("state", "training/odometry/state"),
            String,
            queue_size=1,
            latch=True,
        )
        self._truth_sub = rospy.Subscriber(
            self._input_topic,
            Odometry,
            self._truth_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self._diagnostics_timer = rospy.Timer(
            rospy.Duration(1.0 / self._diagnostics_rate),
            self._diagnostics_callback,
        )
        self._publish_metadata()
        rospy.logwarn(
            "Training truth odometry active: %s -> %s; source=%s, "
            "twist_expression_frame=%s, no FAST-LIO provenance",
            self._input_topic,
            self._output_topic,
            SOURCE_TYPE,
            VELOCITY_FRAME,
        )

    def _assert_output_has_no_publisher(self):
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        except Exception as error:
            raise rospy.ROSInitException(
                "cannot audit odometry publisher exclusivity: {}".format(error)
            )
        existing = []
        for topic, nodes in publishers:
            if rospy.resolve_name(topic) == self._output_topic:
                existing.extend(nodes)
        if existing:
            raise rospy.ROSInitException(
                "adapted odometry already has publisher(s): {}".format(
                    ",".join(sorted(existing))
                )
            )

    def _output_publishers(self):
        publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        for topic, nodes in publishers:
            if rospy.resolve_name(topic) == self._output_topic:
                return sorted(nodes)
        return []

    @staticmethod
    def _sample(message):
        pose = message.pose.pose
        twist = message.twist.twist
        return TruthOdometrySample(
            stamp_sec=message.header.stamp.to_sec(),
            frame_id=message.header.frame_id,
            child_frame_id=message.child_frame_id,
            position=(pose.position.x, pose.position.y, pose.position.z),
            orientation_xyzw=(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
            linear_velocity_world=(
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
            ),
            angular_velocity_world=(
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ),
        )

    def _truth_callback(self, message):
        callback_start = rospy.Time.now()
        sample = self._sample(message)
        valid, reason = validate_truth_sample(
            sample=sample,
            now_sec=callback_start.to_sec(),
            expected_frame=self._expected_raw_frame,
            expected_child_frame=self._expected_raw_child,
            maximum_age_sec=self._maximum_age,
            future_tolerance_sec=self._future_tolerance,
            quaternion_norm_tolerance=self._quaternion_tolerance,
        )
        with self._lock:
            if not valid:
                self._rejected += 1
                self._last_rejection = reason
                return
            if not self._last_stamp.is_zero():
                if message.header.stamp < self._last_stamp:
                    self._out_of_order += 1
                    self._rejected += 1
                    self._last_rejection = "out_of_order_stamp"
                    return
                if message.header.stamp == self._last_stamp:
                    self._duplicate += 1
                    self._rejected += 1
                    self._last_rejection = "duplicate_stamp"
                    return
                if (
                    self._last_position is not None
                    and vector_difference_norm(sample.position, self._last_position)
                    >= self._teleport_distance
                ):
                    self._teleport_discontinuities += 1

            output = copy.deepcopy(message)
            output.header.frame_id = self._output_frame
            output.child_frame_id = self._output_child
            self._odom_pub.publish(output)
            publish_time = rospy.Time.now()
            self._accepted += 1
            self._last_stamp = message.header.stamp
            self._last_position = sample.position
            self._last_callback_latency = max(
                0.0, publish_time.to_sec() - callback_start.to_sec()
            )
            self._last_rejection = ""

    def _metadata(self):
        return {
            "contract_version": CONTRACT_VERSION,
            "backend_mode": BACKEND_MODE,
            "source_type": SOURCE_TYPE,
            "raw_topic": self._input_topic,
            "output_topic": self._output_topic,
            "pose_expression_frame": normalize_frame(self._output_frame),
            "orientation_convention": "ROS_ENU_world_to_FLU_body_quaternion",
            "linear_velocity_expression_frame": VELOCITY_FRAME,
            "angular_velocity_expression_frame": VELOCITY_FRAME,
            "header_stamp_clock": "ROS_sim_time_from_gazebo_world",
            "frame_id": self._output_frame,
            "child_frame_id": self._output_child,
            "transform": "identity_validated_no_numeric_transform",
            "fast_lio_provenance": False,
        }

    def _publish_metadata(self):
        self._metadata_pub.publish(
            String(data=json.dumps(self._metadata(), sort_keys=True))
        )

    def _diagnostics_callback(self, _event):
        now = rospy.Time.now()
        try:
            output_publishers = self._output_publishers()
        except Exception as error:
            rospy.logfatal("truth odometry exclusivity audit failed: %s", error)
            rospy.signal_shutdown("odometry publisher exclusivity unavailable")
            return
        conflicting_publishers = [
            node for node in output_publishers if node != rospy.get_name()
        ]
        if conflicting_publishers:
            rospy.logfatal(
                "truth/FAST-LIO odometry publisher conflict on %s: %s",
                self._output_topic,
                conflicting_publishers,
            )
            rospy.signal_shutdown("odometry publisher exclusivity violated")
            return
        with self._lock:
            age = (
                math.inf
                if self._last_stamp.is_zero()
                else max(0.0, (now - self._last_stamp).to_sec())
            )
            payload = self._metadata()
            payload.update(
                accepted_count=self._accepted,
                rejected_count=self._rejected,
                out_of_order_count=self._out_of_order,
                duplicate_count=self._duplicate,
                teleport_discontinuity_count=self._teleport_discontinuities,
                last_source_stamp_sec=(
                    0.0 if self._last_stamp.is_zero() else self._last_stamp.to_sec()
                ),
                last_source_age_sec=age,
                last_callback_latency_sec=self._last_callback_latency,
                last_rejection=self._last_rejection,
                output_publishers=output_publishers,
                publisher_exclusive=(output_publishers == [rospy.get_name()]),
                ready=bool(
                    self._accepted
                    and age <= self._maximum_age
                    and output_publishers == [rospy.get_name()]
                ),
            )
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/training_truth_odometry"
        status.hardware_id = SOURCE_TYPE + "(training-only)"
        status.level = (
            DiagnosticStatus.OK if payload["ready"] else DiagnosticStatus.WARN
        )
        status.message = (
            "training_truth_odometry_ready"
            if payload["ready"]
            else payload["last_rejection"] or "waiting_for_truth"
        )
        status.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in sorted(payload.items())
        ]
        array = DiagnosticArray(header=Header(stamp=now), status=[status])
        self._diagnostics_pub.publish(array)


def main():
    rospy.init_node("gazebo_truth_odometry_adapter")
    GazeboTruthOdometryAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
