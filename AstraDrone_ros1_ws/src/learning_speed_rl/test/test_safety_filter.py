#!/usr/bin/env python3

import math
import unittest

from learning_speed_rl.policy.safety_filter import (
    SafetyFilterConfig,
    SpeedSafetyFilter,
)
from learning_speed_rl.policy import FixedSpeedPolicy


def config():
    return SafetyFilterConfig(
        v_max_min=0.1,
        v_max_max=1.0,
        initial_v_max=0.8,
    )


class SpeedSafetyFilterTest(unittest.TestCase):
    def test_configuration_requires_finite_reviewed_range(self):
        with self.assertRaises(ValueError):
            SpeedSafetyFilter(SafetyFilterConfig(math.nan, 1.0, 0.8))
        with self.assertRaises(ValueError):
            SpeedSafetyFilter(SafetyFilterConfig(1.0, 0.1, 0.8))
        with self.assertRaises(ValueError):
            SpeedSafetyFilter(SafetyFilterConfig(0.1, 1.0, 1.1))

    def test_fixed_policy_is_immutable_and_finite(self):
        policy = FixedSpeedPolicy(0.16)
        self.assertAlmostEqual(policy.predict(None), 0.16)
        self.assertAlmostEqual(policy.predict(object()), 0.16)
        with self.assertRaises(ValueError):
            FixedSpeedPolicy(math.nan)
        with self.assertRaises(ValueError):
            FixedSpeedPolicy(0.0)

    def test_clamps_to_reviewed_range(self):
        speed_filter = SpeedSafetyFilter(config())
        self.assertAlmostEqual(speed_filter.filter(-5.0), 0.1)
        self.assertAlmostEqual(speed_filter.filter(5.0), 1.0)

    def test_legal_actions_are_not_dynamically_shaped(self):
        speed_filter = SpeedSafetyFilter(config())
        for requested in (0.2, 0.9, 0.4, 0.4, 1.0, 0.1):
            self.assertAlmostEqual(speed_filter.filter(requested), requested)

    def test_rejects_non_finite_policy_output(self):
        speed_filter = SpeedSafetyFilter(config())
        with self.assertRaises(ValueError):
            speed_filter.filter(math.nan)

    def test_high_speed_fixed_requests_are_not_clamped(self):
        for requested in (1.75, 2.0, 2.5, 3.0, 3.5):
            speed_filter = SpeedSafetyFilter(SafetyFilterConfig(
                v_max_min=0.05,
                v_max_max=4.0,
                initial_v_max=requested,
            ))
            self.assertAlmostEqual(
                speed_filter.filter(requested), requested)


if __name__ == "__main__":
    unittest.main()
