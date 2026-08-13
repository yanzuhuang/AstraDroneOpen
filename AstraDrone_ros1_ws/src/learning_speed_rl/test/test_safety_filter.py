#!/usr/bin/env python3

import math
import unittest

from learning_speed_rl.policy.safety_filter import (
    SafetyFilterConfig,
    SpeedSafetyFilter,
)


def config():
    return SafetyFilterConfig(
        v_max_min=0.1,
        v_max_max=1.0,
        initial_v_max=0.8,
        rise_rate_mps2=0.5,
        fall_rate_mps2=1.0,
        maximum_step_mps=0.2,
        low_pass_alpha=1.0,
        hysteresis_mps=0.01,
    )


class SpeedSafetyFilterTest(unittest.TestCase):
    def test_clamps_and_limits_fall_rate(self):
        speed_filter = SpeedSafetyFilter(config())
        self.assertAlmostEqual(speed_filter.update(-5.0, 0.0), 0.8)
        self.assertAlmostEqual(speed_filter.update(-5.0, 0.1), 0.7)
        self.assertAlmostEqual(speed_filter.update(-5.0, 0.2), 0.6)

    def test_limits_rise_rate_and_maximum_step(self):
        speed_filter = SpeedSafetyFilter(config())
        speed_filter.update(1.0, 0.0)
        self.assertAlmostEqual(speed_filter.update(1.0, 1.0), 1.0)

    def test_hysteresis_retains_previous_target(self):
        speed_filter = SpeedSafetyFilter(config())
        speed_filter.update(0.805, 0.0)
        self.assertAlmostEqual(speed_filter.target, 0.8)

    def test_rejects_non_finite_policy_output(self):
        speed_filter = SpeedSafetyFilter(config())
        with self.assertRaises(ValueError):
            speed_filter.update(math.nan, 0.0)


if __name__ == "__main__":
    unittest.main()
