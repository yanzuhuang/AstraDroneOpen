import math
import unittest
from types import SimpleNamespace

from astra_swarm_manager.policy import (
    advance_entry_owner,
    angular_separation_degrees,
    corridor_clear,
    eligible_entry_ids,
    entry_candidate_allowed,
    entry_owner_orbit_established,
    fixed_layers_clear,
    landing_permissions,
    mission_geometry_clear,
    orbit_phase_hold_ids,
    orbit_release_allowed,
    rotate_xy_about_center,
    scheduled_takeoff_allowed,
    serialized_landing_permissions,
    task_start_barrier_ready,
    transition_permissions,
    uav2_takeoff_allowed,
)


class PolicyTest(unittest.TestCase):
    @staticmethod
    def point(angle_degrees, radius=12.5, center=(-10.0, 20.0)):
        angle = math.radians(angle_degrees)
        return (
            center[0] + radius * math.cos(angle),
            center[1] + radius * math.sin(angle))

    def test_orbit_release_requires_measured_phase_from_every_peer(self):
        center = (-10.0, 20.0)
        waiting = (-10.0, 35.0)
        peer_120 = (-22.990381, 12.5)
        peer_240 = (2.990381, 12.5)
        self.assertAlmostEqual(
            angular_separation_degrees(waiting, peer_120, center),
            120.0, places=4)
        self.assertTrue(orbit_release_allowed(
            waiting, [peer_120, peer_240], center, 120.0, 15.0))
        self.assertFalse(orbit_release_allowed(
            waiting, [(-10.0, 5.0)], center, 120.0, 15.0))
        self.assertTrue(orbit_release_allowed(
            waiting, [], center, 120.0, 15.0))

    def test_projects_peer_angle_for_entry_eta(self):
        projected = rotate_xy_about_center(
            (1.0, 0.0), (0.0, 0.0), math.pi / 2.0, 2.0)
        self.assertAlmostEqual(projected[0], 0.0, places=6)
        self.assertAlmostEqual(projected[1], 2.0, places=6)

    def test_dynamic_phase_hold_stops_only_ccw_trailing_vehicle(self):
        positions = {
            1: self.point(10.0),
            2: self.point(90.0),
            3: self.point(240.0),
        }
        self.assertEqual(
            orbit_phase_hold_ids(
                positions, (-10.0, 20.0), 120.0, 15.0),
            {1})

    def test_dynamic_phase_hold_wraparound_identifies_trailing_vehicle(self):
        positions = {
            1: self.point(350.0),
            2: self.point(80.0),
            3: self.point(220.0),
        }
        self.assertEqual(
            orbit_phase_hold_ids(
                positions, (-10.0, 20.0), 120.0, 15.0),
            {1})

    def test_dynamic_phase_hold_has_release_hysteresis(self):
        positions = {
            1: self.point(0.0),
            2: self.point(110.0),
            3: self.point(240.0),
        }
        self.assertEqual(
            orbit_phase_hold_ids(
                positions, (-10.0, 20.0), 120.0, 15.0),
            set())
        self.assertEqual(
            orbit_phase_hold_ids(
                positions, (-10.0, 20.0), 120.0, 15.0, {1}),
            {1})
        positions[2] = self.point(113.0)
        self.assertEqual(
            orbit_phase_hold_ids(
                positions, (-10.0, 20.0), 120.0, 15.0, {1}),
            set())

    def test_dynamic_phase_hold_ignores_single_remaining_vehicle(self):
        self.assertEqual(
            orbit_phase_hold_ids(
                {3: self.point(90.0)}, (-10.0, 20.0), 120.0, 15.0,
                {3}),
            set())

    def test_entry_owner_releases_only_after_orbit_is_established(self):
        self.assertEqual(
            advance_entry_owner(2, 0, True, True, False), (0, 0))
        self.assertEqual(
            advance_entry_owner(2, 0, False, True, False), (2, 0))

    def test_pre_entry_hold_does_not_establish_orbit(self):
        active = {"TARGET_LOCKED", "NAVIGATING", "HOLDING"}
        self.assertFalse(entry_owner_orbit_established(
            "WAIT_ORBIT_PERMISSION", False, active))
        self.assertFalse(entry_owner_orbit_established(
            "HOLDING", False, active))
        self.assertTrue(entry_owner_orbit_established(
            "TARGET_LOCKED", False, active))
        self.assertTrue(entry_owner_orbit_established(
            "HOLDING", True, active))

    def test_terminal_or_non_hover_vehicle_cannot_receive_entry(self):
        self.assertTrue(entry_candidate_allowed(
            True, "WAIT_ENTRY_PERMISSION", "HOVER_READY"))
        self.assertFalse(entry_candidate_allowed(
            True, "WAIT_ENTRY_PERMISSION", "DONE"))
        self.assertFalse(entry_candidate_allowed(
            True, "WAIT_ENTRY_PERMISSION", "LANDING"))
        self.assertFalse(entry_candidate_allowed(
            False, "WAIT_ENTRY_PERMISSION", "HOVER_READY"))

    def test_entry_candidates_fail_closed_before_first_state(self):
        states = {
            2: SimpleNamespace(
                mission_phase="WAIT_ENTRY_PERMISSION",
                flight_state="HOVER_READY")}
        self.assertEqual(
            eligible_entry_ids(
                [1, 2, 3], states, {1: True, 2: True, 3: True}),
            [2])
        self.assertEqual(eligible_entry_ids([1, 2, 3], {}, {}), [])

    def test_failed_ingress_transfers_to_return_owner(self):
        self.assertEqual(
            advance_entry_owner(2, 0, True, False, True), (0, 2))
        self.assertEqual(
            advance_entry_owner(2, 1, True, False, True), (2, 1))

    def test_fixed_two_three_four_layers_are_rejected(self):
        self.assertFalse(fixed_layers_clear([2.0, 3.0, 4.0], 6.0))
        self.assertTrue(fixed_layers_clear([2.0, 8.0, 14.0], 6.0))

    def test_two_three_four_mission_is_safe_with_phase_separation(self):
        self.assertTrue(mission_geometry_clear(
            [2.0, 3.0, 4.0], [270.0, 45.0, 135.0],
            12.5, 3.0, 1.5))
        self.assertFalse(mission_geometry_clear(
            [2.0, 3.0, 4.0], [270.0, 270.0, 270.0],
            12.5, 3.0, 1.5))

    def test_future_point_blocks_takeoff_corridor(self):
        self.assertFalse(corridor_clear(
            [(4.0, 0.0, 2.0)], (4.0, 0.0, 0.0),
            3.0, 3.0, 1.5))
        self.assertTrue(corridor_clear(
            [(8.0, 0.0, 2.0)], (4.0, 0.0, 0.0),
            3.0, 3.0, 1.5))

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

    def test_task_start_barrier_requires_every_hover_ready(self):
        states = {
            uid: SimpleNamespace(
                mission_phase="WAIT_INPUTS", flight_state="HOVER_READY")
            for uid in (1, 2, 3)
        }
        health = {uid: True for uid in states}
        self.assertTrue(task_start_barrier_ready(
            [1, 2, 3], states, health, True))
        states[3].flight_state = "TAKEOFF"
        self.assertFalse(task_start_barrier_ready(
            [1, 2, 3], states, health, True))
        states[3].flight_state = "HOVER_READY"
        health[2] = False
        self.assertFalse(task_start_barrier_ready(
            [1, 2, 3], states, health, True))

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
    mission_geometry_clear,
