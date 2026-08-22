#!/usr/bin/env python3
"""Verify one atomic adapter action is emitted for every Episode mock request."""

import threading
import unittest
import rospy
import rostest
from learning_speed_rl.msg import SpeedActionStamped, SpeedRequestStamped


class MockRequestDrivenIntegrationTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._actions = []
        self._publisher = rospy.Publisher(
            "learning_speed/mock_v_max", SpeedRequestStamped, queue_size=100
        )
        self._subscriber = rospy.Subscriber(
            "learning_speed/action_stamped",
            SpeedActionStamped,
            self._action,
            queue_size=50,
        )

    def _action(self, message):
        with self._lock:
            self._actions.append(message)

    def test_every_request_has_one_immediate_atomic_action(self):
        requests = list(range(1, 101))
        connection_deadline = rospy.Time.now() + rospy.Duration(3.0)
        rate = rospy.Rate(100)
        while (
            not rospy.is_shutdown()
            and (
                self._publisher.get_num_connections() == 0
                or self._subscriber.get_num_connections() == 0
            )
            and rospy.Time.now() < connection_deadline
        ):
            rate.sleep()
        self.assertGreater(self._publisher.get_num_connections(), 0)
        self.assertGreater(self._subscriber.get_num_connections(), 0)
        rospy.sleep(0.25)
        with self._lock:
            baseline_count = len(self._actions)

        publish_rate = rospy.Rate(100)
        for request_id in requests:
            request = SpeedRequestStamped()
            request.header.stamp = rospy.Time.now()
            request.version = "learning_speed_request_v1.0"
            request.episode_id = "burst_repeated_value"
            request.step_index = request_id - 1
            request.request_id = request_id
            request.requested_v_max = 0.10
            self._publisher.publish(request)
            publish_rate.sleep()

        deadline = rospy.Time.now() + rospy.Duration(2.0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self._lock:
                if len(self._actions) >= baseline_count + len(requests):
                    break
            rate.sleep()

        with self._lock:
            actions = list(self._actions[baseline_count:])
        self.assertEqual(
            len(actions),
            len(requests),
            [action.request_id for action in actions],
        )
        self.assertEqual(
            [action.source_mode for action in actions], ["mock"] * len(requests)
        )
        self.assertEqual(
            [action.request_id for action in actions],
            requests,
        )
        self.assertEqual(
            [round(action.filtered_v_max, 6) for action in actions],
            [0.10] * len(requests),
        )
        self.assertEqual(
            [action.episode_id for action in actions],
            ["burst_repeated_value"] * len(requests),
        )
        self.assertEqual(
            [action.step_index for action in actions], list(range(100))
        )
        stamps = [action.header.stamp.to_sec() for action in actions]
        self.assertTrue(all(stamp > 0.0 for stamp in stamps))
        self.assertTrue(
            all(later > earlier for earlier, later in zip(stamps, stamps[1:]))
        )


if __name__ == "__main__":
    rospy.init_node("mock_request_driven_integration_test")
    rostest.rosrun(
        "learning_speed_rl",
        "mock_request_driven_integration",
        MockRequestDrivenIntegrationTest,
    )
