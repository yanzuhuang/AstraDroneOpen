#!/usr/bin/env python3

import math
import unittest

from hector_ego_training_backend.adapter_contract import (
    CommandSample,
    StateSample,
    command_is_valid,
    quaternion_from_yaw,
    trajectory_id_is_new,
    tracking_errors,
)


def sample(**overrides):
    values = dict(
        stamp_sec=10.0,
        frame_id="world",
        trajectory_id=7,
        trajectory_ready=True,
        position=(1.0, 2.0, 3.0),
        velocity=(0.5, 0.0, -0.1),
        acceleration=(0.2, -0.3, 0.0),
        yaw=0.4,
        yaw_dot=-0.2,
    )
    values.update(overrides)
    return CommandSample(**values)


class AdapterContractTest(unittest.TestCase):
    def test_valid_command(self):
        valid, reason = command_is_valid(sample(), "world", 10.05, 0.15, 0.02)
        self.assertTrue(valid)
        self.assertEqual(reason, "")

    def test_rejects_wrong_frame_stale_future_and_nonfinite(self):
        cases = (
            (sample(frame_id="map"), 10.0, "frame_mismatch"),
            (sample(stamp_sec=9.0), 10.0, "stale_stamp"),
            (sample(stamp_sec=10.1), 10.0, "future_stamp"),
            (sample(acceleration=(math.nan, 0.0, 0.0)), 10.0, "non_finite"),
        )
        for command, now, expected in cases:
            with self.subTest(expected=expected):
                valid, reason = command_is_valid(
                    command, "world", now, 0.15, 0.02
                )
                self.assertFalse(valid)
                self.assertEqual(reason, expected)

    def test_yaw_quaternion(self):
        self.assertEqual(quaternion_from_yaw(0.0), (0.0, 0.0, 0.0, 1.0))
        _, _, z, w = quaternion_from_yaw(math.pi)
        self.assertAlmostEqual(z, 1.0)
        self.assertAlmostEqual(w, 0.0, places=7)

    def test_trajectory_generation_must_strictly_advance(self):
        self.assertFalse(trajectory_id_is_new(8, 8))
        self.assertFalse(trajectory_id_is_new(7, 8))
        self.assertFalse(trajectory_id_is_new(0, 0))
        self.assertTrue(trajectory_id_is_new(9, 8))

    def test_tracking_errors_keep_planned_and_actual_distinct(self):
        command = sample(position=(1.0, 2.0, 1.5), velocity=(0.5, 0.1, 0.0))
        state = StateSample(position=(0.8, 2.1, 1.4), velocity=(0.4, 0.0, 0.0))
        position, velocity, position_norm, velocity_norm = tracking_errors(
            command, state
        )
        for actual, expected in zip(position, (0.2, -0.1, 0.1)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(velocity, (0.1, 0.1, 0.0)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(position_norm, math.sqrt(0.06))
        self.assertAlmostEqual(velocity_norm, math.sqrt(0.02))


if __name__ == "__main__":
    unittest.main()
