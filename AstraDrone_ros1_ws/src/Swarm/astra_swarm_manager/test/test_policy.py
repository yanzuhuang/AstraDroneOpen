import unittest

from astra_swarm_manager.policy import (
    fixed_layers_clear,
    landing_permissions,
    scheduled_takeoff_allowed,
    serialized_landing_permissions,
    transition_permissions,
    uav2_takeoff_allowed,
)


class PolicyTest(unittest.TestCase):
    def test_fixed_two_three_four_layers_are_rejected(self):
        self.assertFalse(fixed_layers_clear([2.0, 3.0, 4.0], 6.0))
        self.assertTrue(fixed_layers_clear([2.0, 8.0, 14.0], 6.0))

    def test_shared_takeoff_clock_supports_zero_and_three_seconds(self):
        common = (True, True, True, True)
        self.assertTrue(scheduled_takeoff_allowed(
            0, 0.0, 3.0, *common))
        self.assertFalse(scheduled_takeoff_allowed(
            1, 2.99, 3.0, *common))
        self.assertTrue(scheduled_takeoff_allowed(
            1, 3.0, 3.0, *common))
        self.assertTrue(scheduled_takeoff_allowed(
            2, 6.0, 3.0, *common))
        self.assertTrue(scheduled_takeoff_allowed(
            2, 0.0, 0.0, *common))

    def test_three_vehicle_landing_is_serialized(self):
        grants, owner = serialized_landing_permissions([1, 2, 3], 0)
        self.assertEqual(owner, 1)
        self.assertEqual(grants, {1: True, 2: False, 3: False})
        grants, owner = serialized_landing_permissions([2, 3], 1)
        self.assertEqual(owner, 2)
        self.assertEqual(grants, {2: True, 3: False})

    def test_takeoff_delay_is_not_only_condition(self):
        self.assertFalse(uav2_takeoff_allowed(
            6.1, 6.0, False, (0, 0, 5), (2, 0, 0), 3.0, True))
        self.assertFalse(uav2_takeoff_allowed(
            5.9, 6.0, True, (0, 0, 5), (2, 0, 0), 3.0, True))
        self.assertTrue(uav2_takeoff_allowed(
            6.1, 6.0, True, (0, 0, 8), (2, 0, 0), 3.0, True))

    def test_takeoff_checks_full_vertical_corridor(self):
        self.assertFalse(uav2_takeoff_allowed(
            6.1, 6.0, True, (0, 0, 3.6), (2, 0, 0), 3.0, True,
            4.0, 3.0))
        self.assertTrue(uav2_takeoff_allowed(
            6.1, 6.0, True, (0, 0, 8.0), (2, 0, 0), 3.0, True,
            4.0, 3.0))

    def test_uav2_transition_precedes_uav1(self):
        p1, p2, confirmed = transition_permissions(
            "WAIT_TRANSITION_PERMISSION", "WAIT_TRANSITION_PERMISSION",
            28.0, 24.0, 0.4, True)
        self.assertEqual((p1, p2, confirmed), (False, True, False))
        p1, p2, confirmed = transition_permissions(
            "WAIT_TRANSITION_PERMISSION", "NAVIGATING",
            24.1, 24.0, 0.4, True)
        self.assertEqual((p1, p2, confirmed), (True, False, True))

    def test_climb_through_final_height_is_not_layer_confirmation(self):
        p1, p2, confirmed = transition_permissions(
            "WAIT_TRANSITION_PERMISSION", "SEGMENTED_CLIMB",
            24.1, 24.0, 0.4, True)
        self.assertEqual((p1, p2, confirmed), (False, False, False))

    def test_overlapping_landing_owner_is_sticky(self):
        self.assertEqual(
            landing_permissions(True, True, True, 0), (True, False, 1))
        self.assertEqual(
            landing_permissions(False, True, True, 1), (False, False, 1))


if __name__ == "__main__":
    unittest.main()
