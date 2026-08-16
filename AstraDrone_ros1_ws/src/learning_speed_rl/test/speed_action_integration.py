#!/usr/bin/env python3
"""Verify the additive stamped audit action matches existing Float64 outputs."""

import threading
import unittest

import rospy
import rostest
from learning_speed_rl.msg import SpeedActionStamped
from std_msgs.msg import Float64


class SpeedActionIntegrationTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._raw = []
        self._filtered = []
        self._actions = []
        rospy.Subscriber(
            "learning_speed/raw_v_max", Float64, self._raw_callback, queue_size=20
        )
        rospy.Subscriber(
            "learning_speed/v_max", Float64, self._filtered_callback, queue_size=20
        )
        rospy.Subscriber(
            "learning_speed/action_stamped",
            SpeedActionStamped,
            self._action_callback,
            queue_size=20,
        )

    def _raw_callback(self, message):
        with self._lock:
            self._raw.append(message.data)

    def _filtered_callback(self, message):
        with self._lock:
            self._filtered.append(message.data)

    def _action_callback(self, message):
        with self._lock:
            self._actions.append(message)

    def test_atomic_action_mirrors_existing_chain(self):
        deadline = rospy.Time.now() + rospy.Duration(5.0)
        rate = rospy.Rate(50)
        action = None
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self._lock:
                if self._actions and self._raw and self._filtered:
                    action = self._actions[-1]
                    raw = self._raw[-1]
                    filtered = self._filtered[-1]
                    break
            rate.sleep()
        self.assertIsNotNone(action)
        self.assertGreater(action.header.stamp.to_sec(), 0.0)
        self.assertEqual(action.version, "learning_speed_action_v1.0")
        self.assertEqual(action.source_mode, "fixed")
        self.assertAlmostEqual(action.requested_v_max, 0.12)
        self.assertAlmostEqual(action.filtered_v_max, 0.12)
        self.assertAlmostEqual(raw, action.requested_v_max)
        self.assertAlmostEqual(filtered, action.filtered_v_max)


if __name__ == "__main__":
    rospy.init_node("speed_action_integration_test")
    rostest.rosrun(
        "learning_speed_rl",
        "speed_action_integration",
        SpeedActionIntegrationTest,
    )
