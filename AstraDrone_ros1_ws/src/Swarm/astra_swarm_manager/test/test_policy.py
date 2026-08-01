import math
import unittest
from types import SimpleNamespace

from astra_swarm_manager.policy import (
    advance_entry_owner,
    angular_separation_degrees,
    corridor_clear,
    directed_phase_gap_degrees,
    entry_ready_barrier,
    eligible_entry_ids,
    entry_candidate_allowed,
    entry_owner_orbit_established,
    fixed_layers_clear,
    formation_phase_decision,
    formation_speed_scale_targets,
    joint_entry_corridor_selection,
    landing_permissions,
    mission_geometry_clear,
    normalize_degrees,
    orbit_staging_ready_barrier,
    orbit_phase_hold_ids,
    orbit_release_allowed,
    predicted_pair_clear,
    role_chain_hold_ids,
    rotate_xy_about_center,
    scheduled_takeoff_allowed,
    sequential_orbit_release_allowed,
    serialized_landing_permissions,
    slew_speed_scale,
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

    def test_stage5_gate_angles_and_equal_height_geometry(self):
        self.assertEqual(normalize_degrees(315.0 + 22.5), 337.5)
        self.assertEqual(normalize_degrees(315.0 - 22.5), 292.5)
        self.assertTrue(mission_geometry_clear(
            [3.0, 3.0, 3.0], [292.5, 315.0, 337.5],
            12.5, 3.0, 1.5))

    def test_joint_entry_corridor_selects_ordered_max_clearance_set(self):
        center = (-10.0551, 19.7104)

        def candidate(uid, angle, clearance, suffix):
            def radial(radius):
                radians = math.radians(angle)
                return (center[0] + radius * math.cos(radians),
                        center[1] + radius * math.sin(radians), 3.0)
            return {
                "id": "U{}_{}".format(uid, suffix),
                "angle_deg": angle,
                "pre_radius": 18.0,
                "entry_radius": 15.0,
                "clearance": clearance,
                "pre": radial(18.0),
                "entry": radial(15.0),
                "staging": radial(12.5),
            }

        candidates = {
            3: [candidate(3, 337.5, 1.21, "nominal"),
                candidate(3, 340.0, 1.30, "shift")],
            2: [candidate(2, 315.0, 1.22, "nominal"),
                candidate(2, 317.5, 1.30, "shift")],
            1: [candidate(1, 292.5, 1.23, "nominal"),
                candidate(1, 295.0, 1.30, "shift")],
        }
        selected, reason, diagnostics = joint_entry_corridor_selection(
            candidates, [3, 2, 1], {1: 292.5, 2: 315.0, 3: 337.5},
            center, {1: (0.0, 0.0, 3.0), 2: (4.0, 0.0, 3.0),
                     3: (8.0, 0.0, 3.0)},
            1, 20.0, 30.0, 1.0, 3.0, 1.5)
        self.assertEqual(reason, "OK")
        self.assertEqual(
            [selected[uid]["id"] for uid in (3, 2, 1)],
            ["U3_shift", "U2_shift", "U1_shift"])
        self.assertAlmostEqual(diagnostics["minimum_clearance"], 1.30)
        self.assertEqual(diagnostics["gaps"], [22.5, 22.5])

    def test_joint_entry_corridor_rejects_broken_role_order(self):
        center = (0.0, 0.0)
        candidates = {}
        for uid, angle in ((3, 337.5), (2, 305.0), (1, 292.5)):
            radial = lambda radius, a=angle: (
                radius * math.cos(math.radians(a)),
                radius * math.sin(math.radians(a)), 3.0)
            candidates[uid] = [{
                "id": str(uid), "angle_deg": angle,
                "pre_radius": 18.0, "entry_radius": 15.0,
                "clearance": 1.3, "pre": radial(18.0),
                "entry": radial(15.0), "staging": radial(12.5)}]
        selected, reason, diagnostics = joint_entry_corridor_selection(
            candidates, [3, 2, 1], {1: 292.5, 2: 315.0, 3: 337.5},
            center, {1: (-20.0, -20.0, 3.0),
                     2: (0.0, -20.0, 3.0),
                     3: (20.0, -20.0, 3.0)}, 1)
        self.assertEqual(selected, {})
        self.assertEqual(reason, "NO_JOINT_SAFE_COMBINATION")
        self.assertGreater(diagnostics["role_rejected"], 0)

    def test_inflated_map_margin_accepts_sub_one_metre_raw_distance(self):
        center = (-10.0551, 19.7104)

        def candidate(uid, angle):
            def radial(radius):
                radians = math.radians(angle)
                return (center[0] + radius * math.cos(radians),
                        center[1] + radius * math.sin(radians), 3.0)
            return {
                "id": "U{}_0.945260".format(uid),
                "angle_deg": angle,
                "pre_radius": 18.0,
                "entry_radius": 15.0,
                "clearance": 0.945260,
                "pre": radial(18.0),
                "entry": radial(15.0),
                "staging": radial(12.5),
            }

        candidates = {
            3: [candidate(3, 337.5)],
            2: [candidate(2, 315.0)],
            1: [candidate(1, 292.5)],
        }
        selected, reason, _ = joint_entry_corridor_selection(
            candidates, [3, 2, 1], {1: 292.5, 2: 315.0, 3: 337.5},
            center, {1: (0.0, 0.0, 3.0), 2: (4.0, 0.0, 3.0),
                     3: (8.0, 0.0, 3.0)}, 1)
        self.assertEqual(reason, "OK")
        self.assertEqual(set(selected), {1, 2, 3})

    def test_orbit_staging_barrier_accepts_supervised_position_latch(self):
        def state(uid):
            return SimpleNamespace(
                mission_phase="ORBIT_STAGING_READY", current_height=3.0,
                flight_state="HOLD",
                velocity=SimpleNamespace(x=0.02 * uid, y=0.0, z=0.0))
        states = {uid: state(uid) for uid in (1, 2, 3)}
        ready, reasons = orbit_staging_ready_barrier(
            [1, 2, 3], states, {1: True, 2: True, 3: True},
            [3.0, 3.0, 3.0], 0.35, 0.2, True)
        self.assertTrue(ready)
        self.assertEqual(reasons, {1: "READY", 2: "READY", 3: "READY"})

    def test_sequential_release_requires_phase_forward_motion_and_clearance(self):
        center = (0.0, 0.0)
        follower = self.point(337.5, center=center)
        leader = self.point(45.0, center=center)
        angle = math.radians(45.0)
        leader_velocity = (-0.10 * math.sin(angle),
                           0.10 * math.cos(angle))
        prediction1 = [(leader[0], leader[1], 3.0)] * 4
        prediction2 = [(follower[0], follower[1], 3.0)] * 4
        clear = predicted_pair_clear(
            (leader[0], leader[1], 3.0),
            (follower[0], follower[1], 3.0),
            prediction1, prediction2, 3.0, 1.5)
        allowed, phase, speed, conditions = sequential_orbit_release_allowed(
            follower, leader, leader_velocity, center, 1,
            65.0, 70.0, 0.03, True, True, clear, True)
        self.assertTrue(allowed)
        self.assertAlmostEqual(phase, 67.5)
        self.assertAlmostEqual(speed, 0.10)
        self.assertTrue(all(conditions.values()))

        allowed, _, speed, conditions = sequential_orbit_release_allowed(
            follower, leader, (-leader_velocity[0], -leader_velocity[1]),
            center, 1, 65.0, 70.0, 0.03, True, True, clear, True)
        self.assertFalse(allowed)
        self.assertLess(speed, 0.0)
        self.assertFalse(conditions["leader_forward_stable"])

    def test_role_bound_phase_never_infers_uav1_as_leader(self):
        positions = {
            3: self.point(22.5),
            2: self.point(315.0),
            1: self.point(247.5),
        }
        holds, gaps, bands, reason = formation_phase_decision(
            positions, (-10.0, 20.0), [3, 2, 1], 1,
            57.5, 77.5, 45.0, 45.0, 95.0)
        self.assertEqual(reason, "OK")
        self.assertEqual(holds, set())
        self.assertAlmostEqual(gaps["3-2"], 67.5)
        self.assertAlmostEqual(gaps["2-1"], 67.5)
        self.assertEqual(bands, {"3-2": "NORMAL", "2-1": "NORMAL"})
        self.assertAlmostEqual(directed_phase_gap_degrees(
            positions[2], positions[3], (-10.0, 20.0), 1), 67.5)

    def test_warning_phase_generates_smooth_follower_scale(self):
        scales, reason = formation_speed_scale_targets(
            [3, 2, 1], {"3-2": 50.0, "2-1": 67.5}, set(),
            45.0, 57.5, 0.35)
        self.assertEqual(reason, "OK")
        self.assertAlmostEqual(scales[3], 1.0)
        self.assertGreater(scales[2], 0.35)
        self.assertLess(scales[2], 1.0)
        self.assertAlmostEqual(scales[1], 1.0)

    def test_phase_hold_overrides_speed_scale(self):
        scales, reason = formation_speed_scale_targets(
            [3, 2, 1], {"3-2": 67.5, "2-1": 67.5}, {2, 1},
            45.0, 57.5, 0.35)
        self.assertEqual(reason, "OK")
        self.assertEqual(scales, {3: 1.0, 2: 0.0, 1: 0.0})

    def test_speed_scale_recovery_is_rate_limited(self):
        self.assertAlmostEqual(
            slew_speed_scale(0.35, 1.0, 0.1, 0.2, 1.5), 0.37)
        self.assertAlmostEqual(
            slew_speed_scale(1.0, 0.0, 0.1, 0.2, 1.5), 0.85)

    def test_role_bound_emergency_hold_propagates_backward(self):
        positions = {
            3: self.point(20.0),
            2: self.point(340.0),  # only 40 deg behind UAV3
            1: self.point(270.0),
        }
        holds, gaps, bands, reason = formation_phase_decision(
            positions, (-10.0, 20.0), [3, 2, 1], 1,
            57.5, 77.5, 45.0, 45.0, 95.0)
        self.assertEqual(reason, "OK")
        self.assertAlmostEqual(gaps["3-2"], 40.0)
        self.assertEqual(bands["3-2"], "EMERGENCY")
        self.assertEqual(holds, {2, 1})

    def test_direct_hold_propagates_only_downstream_by_fixed_role(self):
        self.assertEqual(role_chain_hold_ids([3, 2, 1], 3), {2, 1})
        self.assertEqual(role_chain_hold_ids([3, 2, 1], 2), {1})
        self.assertEqual(role_chain_hold_ids([3, 2, 1], 1), set())
        self.assertEqual(role_chain_hold_ids([3, 2, 1], 99), set())

    def test_entry_ready_barrier_reports_late_vehicle(self):
        def state(phase, height=3.0, speed=0.0):
            return SimpleNamespace(
                mission_phase=phase, current_height=height,
                flight_state="HOVER_READY",
                velocity=SimpleNamespace(x=speed, y=0.0, z=0.0))
        states = {1: state("ENTRY_READY"), 2: state("ENTRY_READY"),
                  3: state("ENTRY_GATE_TRANSIT")}
        ready, reasons = entry_ready_barrier(
            [1, 2, 3], states, {1: True, 2: True, 3: True},
            3.0, 0.35, 0.2, True)
        self.assertFalse(ready)
        self.assertEqual(reasons[1], "READY")
        self.assertEqual(reasons[3],
                         "NOT_AT_ENTRY_GATE:ENTRY_GATE_TRANSIT")
        states[3] = state("ENTRY_READY")
        ready, reasons = entry_ready_barrier(
            [1, 2, 3], states, {1: True, 2: True, 3: True},
            3.0, 0.35, 0.2, True)
        self.assertTrue(ready)

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
