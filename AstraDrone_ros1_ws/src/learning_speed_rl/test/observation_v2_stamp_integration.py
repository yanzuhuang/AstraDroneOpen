#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from nav_msgs.msg import Odometry
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from learning_speed_rl.msg import LidarSurrogateStamped


class ObservationV2StampIntegrationTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.RLock()
        self._outputs = []
        self._output_sub = rospy.Subscriber(
            "learning_speed/observation_v2/stamped",
            LidarSurrogateStamped,
            self._output,
            queue_size=10,
        )
        self._odom_pub = rospy.Publisher("Odometry", Odometry, queue_size=10)
        self._cloud_pub = rospy.Publisher(
            "cloud", PointCloud2, queue_size=10
        )
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if (
                self._odom_pub.get_num_connections() > 0
                and self._cloud_pub.get_num_connections() > 0
            ):
                break
            rospy.sleep(0.02)
        self.assertGreater(self._odom_pub.get_num_connections(), 0)
        self.assertGreater(self._cloud_pub.get_num_connections(), 0)

    def _output(self, message):
        with self._lock:
            self._outputs.append(message)

    def test_original_sec_nsec_pair_is_preserved(self):
        now = rospy.Time.now()
        source_stamp = rospy.Time(now.secs + 1, 734749999)
        odom = Odometry()
        odom.header.stamp = source_stamp
        odom.header.frame_id = "camera_init"
        odom.child_frame_id = "body"
        odom.pose.pose.orientation.w = 1.0
        cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=source_stamp, frame_id="camera_init"),
            [(2.0, 0.0, 0.5)],
        )
        for _ in range(5):
            self._odom_pub.publish(odom)
            rospy.sleep(0.02)
            self._cloud_pub.publish(cloud)
            rospy.sleep(0.05)
        deadline = time.time() + 5.0
        output = None
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                output = next(
                    (message for message in reversed(self._outputs) if message.valid),
                    None,
                )
            if output is not None:
                break
            rospy.sleep(0.02)
        self.assertIsNotNone(output)
        self.assertEqual(output.header.stamp.secs, source_stamp.secs)
        self.assertEqual(output.header.stamp.nsecs, source_stamp.nsecs)
        self.assertEqual(output.header.stamp.to_nsec(), source_stamp.to_nsec())

        invalid_stamp = rospy.Time(source_stamp.secs + 1, source_stamp.nsecs)
        invalid_cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=invalid_stamp, frame_id="camera_init"),
            [(2.0, 0.0, 0.5)],
        )
        self._cloud_pub.publish(invalid_cloud)
        invalid_output = None
        deadline = time.time() + 5.0
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                invalid_output = next(
                    (
                        message for message in reversed(self._outputs)
                        if not message.valid
                        and "timestamp_sync:cloud_newer_than_pose_history"
                        in message.diagnostics
                    ),
                    None,
                )
            if invalid_output is not None:
                break
            rospy.sleep(0.02)
        self.assertIsNotNone(invalid_output)
        self.assertEqual(invalid_output.header.stamp.secs, invalid_stamp.secs)
        self.assertEqual(invalid_output.header.stamp.nsecs, invalid_stamp.nsecs)


if __name__ == "__main__":
    rospy.init_node("observation_v2_stamp_integration_test")
    rostest.rosrun(
        "learning_speed_rl",
        "observation_v2_stamp_integration",
        ObservationV2StampIntegrationTest,
    )
