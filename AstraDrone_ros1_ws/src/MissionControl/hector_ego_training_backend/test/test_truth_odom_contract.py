#!/usr/bin/env python3

import math
import unittest

from hector_ego_training_backend.truth_odom_contract import (
    TruthOdometrySample,
    validate_truth_sample,
    wrapped_angle_difference,
)


def sample(**overrides):
    values = dict(
        stamp_sec=10.0,
        frame_id="world",
        child_frame_id="base_link",
        position=(1.0, 2.0, 1.5),
        orientation_xyzw=(0.0, 0.0, 0.0, 1.0),
        linear_velocity_world=(0.4, -0.1, 0.0),
        angular_velocity_world=(0.0, 0.0, 0.2),
    )
    values.update(overrides)
    return TruthOdometrySample(**values)


def validate(value, now=10.01):
    return validate_truth_sample(
        value, now, "world", "base_link", 0.05, 0.01, 0.001
    )


class TruthOdometryContractTest(unittest.TestCase):
    def test_accepts_exact_identity_aligned_truth(self):
        self.assertEqual(validate(sample()), (True, ""))
        self.assertEqual(validate(sample(frame_id="/world")), (True, ""))

    def test_rejects_frame_child_stamp_and_nonfinite_failures(self):
        cases = (
            (sample(frame_id="camera_init"), 10.01, "frame_mismatch"),
            (sample(child_frame_id="body"), 10.01, "child_frame_mismatch"),
            (sample(stamp_sec=9.0), 10.01, "stale_stamp"),
            (sample(stamp_sec=10.1), 10.01, "future_stamp"),
            (
                sample(linear_velocity_world=(math.nan, 0.0, 0.0)),
                10.01,
                "non_finite",
            ),
        )
        for value, now, expected in cases:
            with self.subTest(expected=expected):
                self.assertEqual(validate(value, now), (False, expected))

    def test_rejects_nonunit_orientation(self):
        self.assertEqual(
            validate(sample(orientation_xyzw=(0.0, 0.0, 0.0, 2.0))),
            (False, "invalid_quaternion_norm"),
        )

    def test_yaw_difference_wraps(self):
        self.assertAlmostEqual(
            wrapped_angle_difference(-math.pi + 0.1, math.pi - 0.1), 0.2
        )


if __name__ == "__main__":
    unittest.main()
