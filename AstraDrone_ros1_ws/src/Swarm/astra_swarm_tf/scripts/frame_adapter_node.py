#!/usr/bin/env python3
"""Rewrite hard-coded FAST-LIO frames at the swarm integration boundary."""

import copy

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage


class FrameAdapter:
    def __init__(self):
        self.prefix = rospy.get_param("~frame_prefix").strip("/")
        self.planning_frame = self.prefixed("camera_init")
        self.body_frame = self.prefixed("body")
        self.map_frame = self.prefixed("map")
        self.odom_pub = rospy.Publisher(
            rospy.get_param("~odom_output"), Odometry, queue_size=5)
        self.cloud_pub = rospy.Publisher(
            rospy.get_param("~cloud_output"), PointCloud2, queue_size=2)
        self.pose_pub = rospy.Publisher(
            rospy.get_param("~mavros_pose_output"), PoseStamped, queue_size=5)
        mavros_odom_output = rospy.get_param("~mavros_odom_output", "")
        self.mavros_odom_pub = (
            rospy.Publisher(mavros_odom_output, Odometry, queue_size=5)
            if mavros_odom_output else None)
        self.tf_pub = rospy.Publisher(
            rospy.get_param("~tf_output", "/tf"), TFMessage, queue_size=20)
        rospy.Subscriber(rospy.get_param("~odom_input"), Odometry,
                         self.odom_cb, queue_size=5)
        rospy.Subscriber(rospy.get_param("~cloud_input"), PointCloud2,
                         self.cloud_cb, queue_size=2)
        rospy.Subscriber(rospy.get_param("~mavros_pose_input"), PoseStamped,
                         self.pose_cb, queue_size=5)
        rospy.Subscriber(rospy.get_param("~tf_input"), TFMessage,
                         self.tf_cb, queue_size=20)

    def prefixed(self, frame):
        frame = frame.lstrip("/")
        if frame.startswith(self.prefix + "/"):
            return frame
        return self.prefix + "/" + frame

    def odom_cb(self, msg):
        output = copy.deepcopy(msg)
        output.header.frame_id = self.planning_frame
        output.child_frame_id = self.body_frame
        self.odom_pub.publish(output)
        if self.mavros_odom_pub is not None:
            # MAVROS' odometry plugin uses these configured frame names to
            # perform the ENU/FLU -> NED/FRD conversion before emitting the
            # MAVLink ODOMETRY message. Keep the names local to MAVROS; the
            # prefixed copy above remains the ROS swarm/planning contract.
            mavros_odom = copy.deepcopy(msg)
            mavros_odom.header.frame_id = "odom"
            mavros_odom.child_frame_id = "base_link"
            self.mavros_odom_pub.publish(mavros_odom)

    def cloud_cb(self, msg):
        output = copy.deepcopy(msg)
        output.header.frame_id = self.planning_frame
        self.cloud_pub.publish(output)

    def pose_cb(self, msg):
        output = copy.deepcopy(msg)
        output.header.frame_id = self.map_frame
        self.pose_pub.publish(output)

    def tf_cb(self, msg):
        output = TFMessage()
        for transform in msg.transforms:
            rewritten = copy.deepcopy(transform)
            rewritten.header.frame_id = self.prefixed(
                rewritten.header.frame_id)
            rewritten.child_frame_id = self.prefixed(
                rewritten.child_frame_id)
            output.transforms.append(rewritten)
        if output.transforms:
            self.tf_pub.publish(output)


if __name__ == "__main__":
    rospy.init_node("swarm_frame_adapter")
    FrameAdapter()
    rospy.spin()
