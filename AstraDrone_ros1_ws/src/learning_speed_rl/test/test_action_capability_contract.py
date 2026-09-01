#!/usr/bin/env python3

import unittest

from learning_speed_rl.training.sac_replay import (
    ActionMapping,
    validate_action_range_with_capability,
)


class ActionCapabilityContractTest(unittest.TestCase):
    def test_sac_subset_of_filter_capability_passes(self):
        validate_action_range_with_capability(0.30, 1.75, 0.30, 4.00)

    def test_sac_equal_to_filter_capability_passes(self):
        validate_action_range_with_capability(0.30, 4.00, 0.30, 4.00)

    def test_sac_min_below_filter_capability_fails(self):
        with self.assertRaisesRegex(ValueError, "outside downstream"):
            validate_action_range_with_capability(0.20, 1.75, 0.30, 4.00)

    def test_sac_max_above_filter_capability_fails(self):
        with self.assertRaisesRegex(ValueError, "outside downstream"):
            validate_action_range_with_capability(0.30, 4.50, 0.30, 4.00)

    def test_sac_mapping_uses_configured_endpoints(self):
        mapping = ActionMapping(0.30, 1.75)
        self.assertAlmostEqual(mapping.to_v_max(-1.0), 0.30, places=12)
        self.assertAlmostEqual(mapping.to_v_max(1.0), 1.75, places=12)

    def test_larger_filter_capability_does_not_change_sac_mapping(self):
        mapping = ActionMapping(0.30, 1.75)
        before = [mapping.to_v_max(value) for value in (-1.0, 0.0, 1.0)]
        validate_action_range_with_capability(0.30, 1.75, 0.30, 4.00)
        validate_action_range_with_capability(0.30, 1.75, 0.30, 5.00)
        after = [mapping.to_v_max(value) for value in (-1.0, 0.0, 1.0)]
        self.assertEqual(before, after)

    def test_filter_capability_shrinking_below_sac_max_fails_clearly(self):
        with self.assertRaisesRegex(
            ValueError,
            r"SAC action range \[0\.300000, 1\.750000\].*"
            r"capability \[0\.300000, 1\.700000\]",
        ):
            validate_action_range_with_capability(0.30, 1.75, 0.30, 1.70)

    def test_endpoint_roundoff_within_tolerance_passes(self):
        validate_action_range_with_capability(
            0.30 - 5.0e-10,
            1.75 + 5.0e-10,
            0.30,
            1.75,
        )


if __name__ == "__main__":
    unittest.main()
