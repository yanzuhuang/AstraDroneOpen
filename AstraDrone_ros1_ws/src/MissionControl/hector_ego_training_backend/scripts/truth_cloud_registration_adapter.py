#!/usr/bin/env python3
"""Register raw training Mid360 points into EGO's world frame using truth odom."""

from collections import deque
import json
import math
import threading
import time

import numpy as np
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger, TriggerResponse

from hector_ego_training_backend.cloud_registration_contract import (
    register_sensor_points_world,
    segment_corridor_statistics,
)


class TruthCloudRegistrationAdapter:
    def __init__(self):
        self._lock = threading.RLock()
        self._sensor_frame = str(rospy.get_param("~expected_sensor_frame", "mid360_link")).lstrip("/")
        self._world_frame = str(rospy.get_param("~world_frame", "world")).lstrip("/")
        self._pose_capacity = int(rospy.get_param("~pose_history_capacity", 400))
        self._pending_capacity = int(rospy.get_param("~pending_cloud_capacity", 5))
        self._maximum_pose_lag = float(rospy.get_param("~maximum_pose_lag", 0.05))
        self._point_stride = int(rospy.get_param("~point_stride", 1))
        self._minimum_range = float(rospy.get_param("~minimum_range", 0.2))
        self._maximum_range = float(rospy.get_param("~maximum_range", 40.0))
        self._sensor_translation = tuple(float(v) for v in rospy.get_param("~sensor_translation", [0.0, 0.0, 0.13]))
        self._route_start = tuple(float(v) for v in rospy.get_param("~route_start", [0.0, 0.0, 3.0]))
        self._route_goal = tuple(float(v) for v in rospy.get_param("~route_goal", [-4.3148485145, 5.8522070123, 3.0]))
        self._corridor_half_height = float(rospy.get_param("~corridor_half_height", 0.75))
        self._corridor_radius = float(rospy.get_param("~corridor_radius", 0.80))
        diagnostics_rate = float(rospy.get_param("~diagnostics_rate", 2.0))
        if (
            self._pose_capacity < 2 or self._pending_capacity < 1
            or self._maximum_pose_lag <= 0.0 or self._point_stride < 1
            or self._minimum_range <= 0.0
            or self._maximum_range <= self._minimum_range
            or len(self._sensor_translation) != 3 or len(self._route_start) != 3
            or len(self._route_goal) != 3 or diagnostics_rate <= 0.0
        ):
            raise rospy.ROSInitException("invalid truth cloud registration configuration")

        self._poses = deque(maxlen=self._pose_capacity)
        self._pending = deque(maxlen=self._pending_capacity)
        self._generation = 0
        self._barrier = 0.0
        self._accepted = 0
        self._rejected = 0
        self._future_pose_count = 0
        self._pending_drop_count = 0
        self._last_failure = ""
        self._last_state = {"ready": False}
        self._output_pub = rospy.Publisher("training/cloud_registered", PointCloud2, queue_size=1)
        self._state_pub = rospy.Publisher("training/cloud_registration/state", String, queue_size=1, latch=True)
        self._diagnostics_pub = rospy.Publisher("training/cloud_registration/diagnostics", DiagnosticArray, queue_size=1)
        rospy.Subscriber("Odometry", Odometry, self._odom_callback, queue_size=100, tcp_nodelay=True)
        rospy.Subscriber("livox/lidar", PointCloud2, self._cloud_callback, queue_size=5)
        self._clear_service = rospy.Service("~clear_temporal_history", Trigger, self._clear_callback)
        self._timer = rospy.Timer(rospy.Duration(1.0 / diagnostics_rate), self._diagnostics_callback)
        rospy.logwarn(
            "Training truth cloud registration active: raw=%s output=%s frame=%s; no FAST-LIO",
            rospy.resolve_name("livox/lidar"), rospy.resolve_name("training/cloud_registered"), self._world_frame,
        )

    @staticmethod
    def _pose(message):
        p = message.pose.pose.position
        q = message.pose.pose.orientation
        return (
            message.header.stamp.to_sec(),
            (p.x, p.y, p.z),
            (q.x, q.y, q.z, q.w),
        )

    def _odom_callback(self, message):
        stamp, position, quaternion = self._pose(message)
        if stamp <= 0.0 or message.header.frame_id.lstrip("/") != self._world_frame:
            return
        pending = []
        with self._lock:
            if stamp <= self._barrier + 1.0e-9:
                return
            if self._poses and stamp <= self._poses[-1][0]:
                return
            self._poses.append((stamp, position, quaternion))
            while self._pending and self._pending[0].header.stamp.to_sec() <= stamp + 1.0e-9:
                pending.append(self._pending.popleft())
        for cloud in pending:
            self._process_cloud(cloud, allow_pending=False)

    def _cloud_callback(self, message):
        self._process_cloud(message, allow_pending=True)

    def _select_pose_locked(self, stamp):
        for pose in reversed(self._poses):
            if pose[0] <= stamp + 1.0e-9:
                return pose
        return None

    def _process_cloud(self, message, allow_pending):
        start = time.monotonic()
        stamp = message.header.stamp.to_sec()
        if stamp <= self._barrier + 1.0e-9 or message.header.frame_id.lstrip("/") != self._sensor_frame:
            with self._lock:
                self._rejected += 1
                self._last_failure = "cloud_barrier_or_frame"
            return
        with self._lock:
            pose = self._select_pose_locked(stamp)
            latest_stamp = self._poses[-1][0] if self._poses else 0.0
            if pose is None and allow_pending and latest_stamp < stamp:
                if len(self._pending) == self._pending.maxlen:
                    self._pending_drop_count += 1
                self._pending.append(message)
                return
        if pose is None:
            with self._lock:
                self._rejected += 1
                self._last_failure = "causal_pose_unavailable"
            return
        pose_stamp, position, quaternion = pose
        lag = stamp - pose_stamp
        if lag < -1.0e-9:
            with self._lock:
                self._future_pose_count += 1
                self._rejected += 1
                self._last_failure = "future_pose_rejected"
            return
        if lag > self._maximum_pose_lag + 1.0e-9:
            with self._lock:
                self._rejected += 1
                self._last_failure = "causal_pose_stale"
            return
        decoded = []
        try:
            for index, point in enumerate(point_cloud2.read_points(message, field_names=("x", "y", "z"), skip_nans=True)):
                if index % self._point_stride != 0 or not all(
                    math.isfinite(float(v)) for v in point[:3]
                ):
                    continue
                range_m = math.sqrt(sum(float(v) * float(v) for v in point[:3]))
                if self._minimum_range <= range_m <= self._maximum_range:
                    decoded.append(point[:3])
        except (KeyError, ValueError) as error:
            with self._lock:
                self._rejected += 1
                self._last_failure = "decode:" + str(error)
            return
        if not decoded:
            with self._lock:
                self._rejected += 1
                self._last_failure = "empty_cloud"
            return
        points_world = register_sensor_points_world(decoded, position, quaternion, self._sensor_translation)
        header = Header(stamp=message.header.stamp, frame_id=self._world_frame)
        self._output_pub.publish(point_cloud2.create_cloud_xyz32(header, points_world.astype(np.float32)))
        corridor = segment_corridor_statistics(
            points_world, self._route_start, self._route_goal,
            self._route_start[2], self._corridor_half_height, self._corridor_radius,
        )
        now = rospy.Time.now().to_sec()
        with self._lock:
            self._accepted += 1
            self._last_failure = ""
            self._last_state = {
                "version": "astradrone_training_truth_cloud_v1.0",
                "ready": True,
                "generation": self._generation,
                "reset_barrier_stamp": self._barrier,
                "source_stamp": stamp,
                "receipt_stamp": now,
                "source_age": max(0.0, now - stamp),
                "pose_stamp": pose_stamp,
                "pose_lag": lag,
                "raw_points": int(message.width * message.height),
                "registered_points": int(points_world.shape[0]),
                "output_frame": self._world_frame,
                "corridor_flight_band_points": corridor["flight_band_points"],
                "corridor_points": corridor["corridor_points"],
                "corridor_minimum_distance": corridor["minimum_distance"],
                "build_duration_ms": (time.monotonic() - start) * 1000.0,
            }
            payload = dict(self._last_state)
        self._state_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _clear_callback(self, _request):
        barrier = rospy.Time.now().to_sec()
        if barrier <= 0.0:
            return TriggerResponse(False, "ROS simulation time is not active")
        with self._lock:
            self._poses.clear()
            self._pending.clear()
            self._generation += 1
            self._barrier = barrier
            self._last_state = {"ready": False, "generation": self._generation, "reset_barrier_stamp": barrier}
            generation = self._generation
        self._state_pub.publish(String(data=json.dumps(self._last_state, sort_keys=True)))
        return TriggerResponse(True, "generation={};barrier={:.9f}".format(generation, barrier))

    def _diagnostics_callback(self, _event):
        with self._lock:
            payload = dict(self._last_state)
            payload.update(
                accepted_clouds=self._accepted, rejected_clouds=self._rejected,
                future_pose_count=self._future_pose_count,
                pending_clouds=len(self._pending), pending_drop_count=self._pending_drop_count,
                last_failure=self._last_failure,
            )
        status = DiagnosticStatus()
        status.name = rospy.get_name() + "/truth_cloud_registration"
        status.hardware_id = "gazebo_truth_training"
        status.level = DiagnosticStatus.OK if payload.get("ready", False) else DiagnosticStatus.WARN
        status.message = "ready" if payload.get("ready", False) else str(payload.get("last_failure", "warming"))
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in sorted(payload.items())]
        message = DiagnosticArray()
        message.header.stamp = rospy.Time.now()
        message.status = [status]
        self._diagnostics_pub.publish(message)


def main():
    rospy.init_node("truth_cloud_registration_adapter")
    TruthCloudRegistrationAdapter()
    rospy.spin()


if __name__ == "__main__":
    main()
