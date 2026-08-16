#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64
from traj_utils.msg import Bspline

from learning_speed_rl.msg import LidarSurrogateStamped, ObservationC


class ObservationCIntegrationTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.RLock()
        self._observations = []
        self._valid_values = []
        self._obs_sub = rospy.Subscriber(
            "learning_speed/observation_c", ObservationC,
            self._observation_callback, queue_size=20,
        )
        self._valid_sub = rospy.Subscriber(
            "learning_speed/observation_c/valid", Bool,
            self._valid_callback, queue_size=20,
        )
        self._odom_pub = rospy.Publisher("Odometry", Odometry, queue_size=20)
        self._trajectory_pub = rospy.Publisher(
            "planning/bspline", Bspline, queue_size=10
        )
        self._lidar_pub = rospy.Publisher(
            "learning_speed/observation_v2/stamped",
            LidarSurrogateStamped, queue_size=10,
        )
        self._lidar_valid_pub = rospy.Publisher(
            "learning_speed/observation_v2/valid", Bool, queue_size=2, latch=True
        )
        self._vmax_pub = rospy.Publisher(
            "learning_speed/applied_v_max", Float64, queue_size=2, latch=True
        )
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if (
                self._odom_pub.get_num_connections() > 0
                and self._trajectory_pub.get_num_connections() > 0
                and self._lidar_pub.get_num_connections() > 0
                and self._vmax_pub.get_num_connections() > 0
            ):
                break
            rospy.sleep(0.02)
        self.assertGreater(self._trajectory_pub.get_num_connections(), 0)

    def _observation_callback(self, message):
        with self._lock:
            self._observations.append(message)

    def _valid_callback(self, message):
        with self._lock:
            self._valid_values.append(bool(message.data))

    def _wait_for(self, predicate, timeout=4.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                if predicate():
                    return True
            rospy.sleep(0.02)
        return False

    @staticmethod
    def _trajectory(start, trajectory_id, y=0.0):
        message = Bspline()
        message.order = 3
        message.traj_id = trajectory_id
        message.start_time = start
        message.frame_id = "camera_init"
        message.drone_id = 0
        message.knots = [float(value) for value in range(-3, 9)]
        message.pos_pts = [Point(x=float(index), y=y, z=0.0) for index in range(8)]
        return message

    @staticmethod
    def _odom(stamp, x, y=0.0):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "camera_init"
        message.child_frame_id = "body"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.w = 1.0
        return message

    @staticmethod
    def _lidar(stamp):
        message = LidarSurrogateStamped()
        message.header.stamp = stamp
        message.header.frame_id = "body"
        message.version = "lidar_surrogate_v2.0"
        message.lidar_surrogate = [5.0] * 16
        message.lidar_valid_mask = [1.0] * 16
        message.unknown_mask = [0.0] * 16
        message.semantic = [1] * 16
        return message

    def _publish_repeated(self, publisher, message, count=3):
        for _ in range(count):
            publisher.publish(message)
            rospy.sleep(0.03)

    def test_valid_replan_stale_rejection_and_fail_closed(self):
        base = rospy.Time.now() + rospy.Duration(0.2)
        self._vmax_pub.publish(Float64(data=0.20))
        self._lidar_valid_pub.publish(Bool(data=True))
        self._publish_repeated(self._trajectory_pub, self._trajectory(base, 1))
        self._odom_pub.publish(self._odom(base, 1.0))
        stamp_one = base + rospy.Duration(0.1)
        self._odom_pub.publish(self._odom(stamp_one, 1.1))
        rospy.sleep(0.05)
        self._publish_repeated(self._lidar_pub, self._lidar(stamp_one))
        self.assertTrue(
            self._wait_for(
                lambda: any(message.valid and message.trajectory_id == 1 for message in self._observations)
            ),
            "observations={}".format(
                [(message.valid, message.trajectory_id, list(message.diagnostics))
                 for message in self._observations]
            ),
        )
        first = next(
            message for message in reversed(self._observations)
            if message.valid and message.trajectory_id == 1
        )
        self.assertEqual(first.header.frame_id, "body")
        self.assertEqual(len(first.future_positions_body), 20)
        self.assertGreater(first.future_positions_body[0].x, 0.0)
        self.assertAlmostEqual(first.actual_velocity_body.x, 1.0, places=4)
        self.assertAlmostEqual(first.previous_v_max, 0.20, places=5)

        start_two = base + rospy.Duration(0.2)
        self._publish_repeated(
            self._trajectory_pub, self._trajectory(start_two, 2, y=2.0)
        )
        self._odom_pub.publish(self._odom(start_two, 1.0, y=2.0))
        stamp_two = start_two + rospy.Duration(0.1)
        self._odom_pub.publish(self._odom(stamp_two, 1.1, y=2.0))
        rospy.sleep(0.05)
        self._publish_repeated(self._lidar_pub, self._lidar(stamp_two))
        self.assertTrue(self._wait_for(
            lambda: any(message.valid and message.trajectory_id == 2 for message in self._observations)
        ))

        # Re-delivery of the older formal trajectory must not restore the cache.
        self._publish_repeated(self._trajectory_pub, self._trajectory(base, 1))
        stamp_three = start_two + rospy.Duration(0.2)
        self._odom_pub.publish(self._odom(stamp_three, 1.2, y=2.0))
        self._publish_repeated(self._lidar_pub, self._lidar(stamp_three))
        self.assertTrue(self._wait_for(
            lambda: bool(self._observations) and self._observations[-1].valid
            and self._observations[-1].trajectory_id == 2
        ))

        self._lidar_valid_pub.publish(Bool(data=False))
        self.assertTrue(self._wait_for(
            lambda: bool(self._valid_values) and not self._valid_values[-1]
            and bool(self._observations) and not self._observations[-1].valid
        ))
        self.assertIn("lidar_surrogate_invalid", self._observations[-1].diagnostics)


if __name__ == "__main__":
    rospy.init_node("observation_c_integration_test")
    rostest.rosrun(
        "learning_speed_rl", "observation_c_integration",
        ObservationCIntegrationTest,
    )
