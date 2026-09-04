import math
import unittest
from types import SimpleNamespace

from astra_swarm_manager.policy import (
    advance_entry_owner,
    angular_separation_degrees,
    completed_predecessor_handoff_allowed,
    contiguous_role_segments,
    corridor_clear,
    directed_phase_gap_degrees,
    entry_ready_barrier,
    eligible_entry_ids,
    entry_candidate_allowed,
    entry_owner_orbit_established,
    fixed_layers_clear,
    formation_phase_decision,
    formation_speed_scale_targets,
    incremental_entry_corridor_selection,
    joint_entry_corridor_selection,
    landing_permissions,
    mission_geometry_clear,
    normalize_degrees,
    orbit_staging_ready_barrier,
    orbit_phase_hold_ids,
    orbit_release_allowed,
    predicted_pair_clear,
    reconcile_entry_corridor_commitments,
    role_chain_hold_ids,
    rotate_xy_about_center,
    scheduled_takeoff_allowed,
    sequential_orbit_release_allowed,
    serialized_landing_permissions,
    serialized_transition_permissions,
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

    @staticmethod
    def entry_corridor(uid, angle_degrees, height, suffix="nominal",
                       center=(0.0, 0.0)):
        def radial(radius):
            angle = math.radians(angle_degrees)
            return (center[0] + radius * math.cos(angle),
                    center[1] + radius * math.sin(angle), height)
        path = [radial(18.0), radial(15.0), radial(12.5)]
        return {
            "id": "U{}_{}".format(uid, suffix),
            "angle_deg": angle_degrees,
            "pre_radius": 18.0,
            "entry_radius": 15.0,
            "orbit_staging_radius": 12.5,
            "endpoint_valid": True,
            "path_valid": True,
            "ego_candidate_valid": True,
            "clearance": 1.0,
            "pre": path[0],
            "entry": path[1],
            "staging": path[2],
            "path": path,
        }

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

    def test_three_uav_gate_angles_and_equal_height_geometry(self):
        self.assertEqual(normalize_degrees(315.0 + 22.5), 337.5)
        self.assertEqual(normalize_degrees(315.0 - 22.5), 292.5)
        self.assertTrue(mission_geometry_clear(
            [3.0, 3.0, 3.0], [292.5, 315.0, 337.5],
            12.5, 3.0, 1.5))

    def test_joint_entry_corridor_tier0_beats_clearer_angle_adjustment(self):
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
        self.assertEqual(reason, "OK_TIER_0")
        self.assertEqual(
            [selected[uid]["id"] for uid in (3, 2, 1)],
            ["U3_nominal", "U2_nominal", "U1_nominal"])
        self.assertAlmostEqual(diagnostics["minimum_clearance"], 1.21)
        self.assertEqual(diagnostics["gaps"], [22.5, 22.5])
        self.assertEqual(diagnostics["selected_tier"], 0)
        self.assertEqual(
            diagnostics["candidate_outcomes"]["3"]["U3_shift"]
            ["final_rejection_reason"],
            "LOWER_TIER_SUCCEEDED_BEFORE_CANDIDATE_ELIGIBLE")

    def test_joint_entry_corridor_exhausts_adjusted_12_5_before_14_5(self):
        center = (-10.0551, 19.7104)

        def candidate(uid, angle, radius, clearance, suffix,
                      endpoint_valid=True):
            def radial(value):
                radians = math.radians(angle)
                return (center[0] + value * math.cos(radians),
                        center[1] + value * math.sin(radians), 3.0)
            return {
                "id": "U{}_{}".format(uid, suffix),
                "angle_deg": angle,
                "pre_radius": 18.0,
                "entry_radius": 15.0,
                "orbit_staging_radius": radius,
                "endpoint_clearance": clearance,
                "endpoint_valid": endpoint_valid,
                "path_valid": True,
                "ego_candidate_valid": True,
                "clearance": clearance,
                "pre": radial(18.0),
                "entry": radial(15.0),
                "staging": radial(radius),
            }

        candidates = {
            3: [candidate(3, 337.5, 12.5, 1.0, "nominal"),
                candidate(3, 349.5, 12.5, 0.8, "adjusted"),
                candidate(3, 337.5, 14.5, 5.0, "outer")],
            2: [candidate(2, 315.0, 12.5, 1.0, "nominal"),
                candidate(2, 327.0, 12.5, 0.8, "adjusted"),
                candidate(2, 315.0, 14.5, 5.0, "outer")],
            1: [candidate(1, 292.5, 12.5, 0.1, "nominal",
                          endpoint_valid=False),
                candidate(1, 304.5, 12.5, 0.8, "adjusted"),
                candidate(1, 292.5, 14.5, 5.0, "outer")],
        }
        selected, reason, diagnostics = joint_entry_corridor_selection(
            candidates, [3, 2, 1], {1: 292.5, 2: 315.0, 3: 337.5},
            center, {1: (0.0, 0.0, 3.0), 2: (4.0, 0.0, 3.0),
                     3: (8.0, 0.0, 3.0)}, 1)
        self.assertEqual(reason, "OK_TIER_1")
        self.assertEqual(diagnostics["selected_tier"], 1)
        self.assertEqual(
            [selected[uid]["id"] for uid in (3, 2, 1)],
            ["U3_adjusted", "U2_adjusted", "U1_adjusted"])
        self.assertEqual(diagnostics["tiers"]["0"]["result"],
                         "NO_JOINT_SAFE_COMBINATION")
        self.assertEqual(diagnostics["tiers"]["1"]["result"], "SELECTED")

    def test_joint_entry_corridor_tier2_precedes_clearer_tier3(self):
        center = (0.0, 0.0)

        def candidate(uid, angle, radius, clearance, suffix):
            radians = math.radians(angle)
            radial = lambda value: (
                value * math.cos(radians), value * math.sin(radians), 3.0)
            return {
                "id": "U{}_{}".format(uid, suffix),
                "angle_deg": angle,
                "pre_radius": 18.0,
                "entry_radius": 15.0,
                "orbit_staging_radius": radius,
                "endpoint_valid": True,
                "path_valid": True,
                "ego_candidate_valid": True,
                "clearance": clearance,
                "pre": radial(18.0), "entry": radial(15.0),
                "staging": radial(radius),
            }

        candidates = {}
        for uid, angle in ((3, 337.5), (2, 315.0), (1, 292.5)):
            candidates[uid] = [
                candidate(uid, angle, 12.5, 0.1, "inner"),
                candidate(uid, angle, 14.5, 0.8, "middle"),
                candidate(uid, angle, 16.5, 9.0, "outer")]
        selected, reason, diagnostics = joint_entry_corridor_selection(
            candidates, [3, 2, 1], {1: 292.5, 2: 315.0, 3: 337.5},
            center, {
                uid: (20.0 * math.cos(math.radians(angle)),
                      20.0 * math.sin(math.radians(angle)), 3.0)
                for uid, angle in ((3, 337.5), (2, 315.0), (1, 292.5))},
            1)
        self.assertEqual(reason, "OK_TIER_2")
        self.assertEqual(diagnostics["selected_tier"], 2)
        self.assertEqual(
            [selected[uid]["id"] for uid in (3, 2, 1)],
            ["U3_middle", "U2_middle", "U1_middle"])
        self.assertNotIn("radius_deviation", diagnostics)
        self.assertIn("ingress_length", diagnostics["selected_rank"])

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
        self.assertEqual(reason, "NO_JOINT_SAFE_COMBINATION_ALL_TIERS")
        self.assertGreater(diagnostics["tiers"]["1"]["role_rejected"], 0)

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
        self.assertEqual(reason, "OK_TIER_0")
        self.assertEqual(set(selected), {1, 2, 3})

    def test_incremental_entry_leader_does_not_wait_for_followers(self):
        order = [3, 2, 1]
        candidates = {
            3: [self.entry_corridor(3, 337.5, 22.0)],
        }
        starts = {
            uid: self.entry_corridor(uid, angle, height)["pre"]
            for uid, angle, height in (
                (3, 337.5, 22.0), (2, 315.0, 28.0),
                (1, 292.5, 34.0))}
        selected, grants, reason, diagnostics = (
            incremental_entry_corridor_selection(
                candidates, {}, order,
                {3: 337.5, 2: 315.0, 1: 292.5}, (0.0, 0.0),
                starts, 1))
        self.assertEqual(set(selected), {3})
        self.assertEqual(grants, {3: True, 2: False, 1: False})
        self.assertEqual(reason, "UAV3_OK_TIER_0")
        self.assertEqual(diagnostics["committed_role_order"], [3])

    def test_incremental_uav2_uses_committed_uav3_corridor_and_trajectory(self):
        order = [3, 2, 1]
        leader = self.entry_corridor(3, 337.5, 22.0)
        follower = self.entry_corridor(2, 315.0, 28.0)
        starts = {
            3: leader["pre"], 2: follower["pre"],
            1: self.entry_corridor(1, 292.5, 34.0)["pre"],
        }
        nominal = {3: 337.5, 2: 315.0, 1: 292.5}

        # The selected UAV3 corridor remains fixed, but its current predicted
        # trajectory overlaps UAV2's proposed ingress and must reject UAV2.
        selected, grants, reason, diagnostics = (
            incremental_entry_corridor_selection(
                {2: [follower]}, {3: leader}, order, nominal,
                (0.0, 0.0), starts, 1,
                committed_prediction_paths={3: follower["path"]}))
        self.assertEqual(selected, {3: leader})
        self.assertEqual(grants, {3: True, 2: False, 1: False})
        self.assertEqual(reason,
                         "NO_JOINT_SAFE_COMBINATION_ALL_TIERS")
        self.assertGreater(diagnostics["tiers"]["0"]["conflict_rejected"],
                           0)

        selected, grants, reason, _ = incremental_entry_corridor_selection(
            {2: [follower]}, {3: leader}, order, nominal,
            (0.0, 0.0), starts, 1,
            committed_prediction_paths={3: leader["path"]})
        self.assertEqual(set(selected), {3, 2})
        self.assertEqual(selected[3]["id"], leader["id"])
        self.assertEqual(grants, {3: True, 2: True, 1: False})
        self.assertEqual(reason, "UAV2_OK_TIER_0")

    def test_incremental_uav2_rejects_crossing_committed_uav3_corridor(self):
        order = [3, 2, 1]
        leader = self.entry_corridor(3, 337.5, 22.0)
        follower = self.entry_corridor(2, 315.0, 28.0)
        leader["path"] = [(0.0, 0.0, 22.0), (10.0, 10.0, 22.0)]
        follower["path"] = [(0.0, 10.0, 28.0), (10.0, 0.0, 28.0)]
        starts = {3: leader["path"][0], 2: follower["path"][0],
                  1: self.entry_corridor(1, 292.5, 34.0)["pre"]}
        selected, grants, reason, diagnostics = (
            incremental_entry_corridor_selection(
                {2: [follower]}, {3: leader}, order,
                {3: 337.5, 2: 315.0, 1: 292.5}, (0.0, 0.0),
                starts, 1,
                committed_prediction_paths={
                    3: [(30.0, 30.0, 22.0), (31.0, 31.0, 22.0)]}))
        self.assertEqual(selected, {3: leader})
        self.assertEqual(grants, {3: True, 2: False, 1: False})
        self.assertEqual(reason,
                         "NO_JOINT_SAFE_COMBINATION_ALL_TIERS")
        self.assertGreater(diagnostics["tiers"]["0"]["crossing_rejected"],
                           0)

    def test_incremental_uav1_join_failure_preserves_prefix_until_global_inhibit(self):
        order = [3, 2, 1]
        leader = self.entry_corridor(3, 337.5, 22.0)
        middle = self.entry_corridor(2, 315.0, 28.0)
        trailing = self.entry_corridor(1, 292.5, 34.0)
        committed = {3: leader, 2: middle}
        starts = {3: leader["pre"], 2: middle["pre"],
                  1: trailing["pre"]}
        nominal = {3: 337.5, 2: 315.0, 1: 292.5}
        predictions = {3: leader["path"], 2: middle["path"]}

        selected, grants, reason, _ = incremental_entry_corridor_selection(
            {}, committed, order, nominal, (0.0, 0.0), starts, 1,
            committed_prediction_paths=predictions)
        self.assertEqual(selected, committed)
        self.assertEqual(grants, {3: True, 2: True, 1: False})
        self.assertEqual(reason, "WAITING_FOR_UAV1_CANDIDATES")

        selected, grants, reason, _ = incremental_entry_corridor_selection(
            {1: [trailing]}, committed, order, nominal, (0.0, 0.0),
            starts, 1, committed_prediction_paths=predictions)
        self.assertEqual(set(selected), {3, 2, 1})
        self.assertEqual(grants, {3: True, 2: True, 1: True})
        self.assertEqual(reason, "UAV1_OK_TIER_0")

        selected, grants, reason, _ = incremental_entry_corridor_selection(
            {}, committed, order, nominal, (0.0, 0.0), starts, 1,
            committed_prediction_paths=predictions, globally_clear=False)
        self.assertEqual(selected, committed)
        self.assertEqual(grants, {3: False, 2: False, 1: False})
        self.assertEqual(reason, "GLOBAL_SAFETY_INHIBIT")

    def test_generation_reconciliation_preserves_entered_predecessor(self):
        order = [3, 2, 1]
        leader = self.entry_corridor(3, 337.5, 22.0, "G1")
        middle_g1 = self.entry_corridor(2, 315.0, 28.0, "G1")
        trailing_g1 = self.entry_corridor(1, 292.5, 34.0, "G1")
        selection = {3: leader, 2: middle_g1, 1: trailing_g1}
        commits = {
            uid: {
                "selected_candidate_id": selection[uid]["id"],
                "candidate_generation": 1,
                "committed": True,
                "entered": uid == 3,
                "released": uid == 3,
                "mission_lock_observed": True,
            }
            for uid in order
        }
        reconciled, states, revoked, reason = (
            reconcile_entry_corridor_commitments(
                selection, commits, order,
                {3: 1, 2: 2, 1: 1},
                {3: {leader["id"]},
                 2: {self.entry_corridor(2, 315.0, 28.0, "G2")["id"]},
                 1: {trailing_g1["id"]}},
                {3: leader["id"], 2: "", 1: trailing_g1["id"]},
                safely_entered_ids={3}, released_ids={3}))
        self.assertEqual(set(reconciled), {3})
        self.assertEqual(revoked, [2, 1])
        self.assertEqual(reason, "UAV2_CANDIDATE_GENERATION_CHANGED")
        self.assertTrue(states[3]["committed"])
        self.assertTrue(states[3]["entered"])
        self.assertTrue(states[3]["released"])
        self.assertFalse(states[2]["committed"])
        self.assertFalse(states[1]["committed"])

    def test_generation_reconciliation_reselects_g2_then_rebuilds_uav1(self):
        order = [3, 2, 1]
        nominal = {3: 337.5, 2: 315.0, 1: 292.5}
        leader = self.entry_corridor(3, 337.5, 22.0, "G1")
        middle_g2 = self.entry_corridor(2, 315.0, 28.0, "G2")
        trailing_g1 = self.entry_corridor(1, 292.5, 34.0, "G1")
        starts = {3: leader["pre"], 2: middle_g2["pre"],
                  1: trailing_g1["pre"]}
        selected, grants, reason, _ = incremental_entry_corridor_selection(
            {2: [middle_g2]}, {3: leader}, order, nominal, (0.0, 0.0),
            starts, 1, committed_prediction_paths={3: leader["path"]})
        self.assertEqual([selected[uid]["id"] for uid in (3, 2)],
                         [leader["id"], middle_g2["id"]])
        self.assertEqual(grants, {3: True, 2: True, 1: False})
        self.assertEqual(reason, "UAV2_OK_TIER_0")

        selected, grants, reason, _ = incremental_entry_corridor_selection(
            {1: [trailing_g1]}, selected, order, nominal, (0.0, 0.0),
            starts, 1, committed_prediction_paths={
                3: leader["path"], 2: middle_g2["path"]})
        self.assertEqual([selected[uid]["id"] for uid in order],
                         [leader["id"], middle_g2["id"], trailing_g1["id"]])
        self.assertEqual(grants, {3: True, 2: True, 1: True})
        self.assertEqual(reason, "UAV1_OK_TIER_0")

    def test_generation_reconciliation_detects_disappeared_locked_candidate(self):
        middle = self.entry_corridor(2, 315.0, 28.0, "G1")
        leader = self.entry_corridor(3, 337.5, 22.0, "G1")
        selection = {3: leader, 2: middle}
        commits = {
            3: {"selected_candidate_id": leader["id"],
                "candidate_generation": 1, "committed": True,
                "entered": True, "released": True,
                "mission_lock_observed": True},
            2: {"selected_candidate_id": middle["id"],
                "candidate_generation": 1, "committed": True,
                "entered": False, "released": False,
                "mission_lock_observed": True},
        }
        reconciled, _, revoked, reason = (
            reconcile_entry_corridor_commitments(
                selection, commits, [3, 2, 1], {3: 1, 2: 1},
                {3: {leader["id"]}, 2: set()},
                {3: leader["id"], 2: ""}, {3}, {3}))
        self.assertEqual(set(reconciled), {3})
        self.assertEqual(revoked, [2])
        self.assertEqual(reason, "UAV2_LOCKED_CANDIDATE_DISAPPEARED")

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

    def test_completed_predecessor_handoff_restores_ordered_liveness(self):
        leader = (20.0, 0.0, 22.0)
        middle = (0.0, -20.0, 28.0)
        trailing = (-20.0, 0.0, 34.0)
        phase_allowed, _, _, phase_conditions = (
            sequential_orbit_release_allowed(
                middle[:2], leader[:2], (0.0, 0.0), (0.0, 0.0), 1,
                65.0, 70.0, 0.03, True, True, True, True))
        self.assertFalse(phase_allowed)
        self.assertFalse(phase_conditions["leader_forward_stable"])
        allowed, conditions = completed_predecessor_handoff_allowed(
            middle, leader, True, True, True, True, True, True, 3.0, 1.5)
        self.assertTrue(allowed)
        self.assertTrue(all(conditions.values()))
        released = {3}
        if allowed:
            released.add(2)
        self.assertEqual(released, {3, 2})

        # UAV1 remains downstream until UAV2 itself has completed; role order
        # is not bypassed by the handoff path.
        allowed, conditions = completed_predecessor_handoff_allowed(
            trailing, middle, False, True, True, True, True, True, 3.0, 1.5)
        self.assertFalse(allowed)
        self.assertFalse(conditions["predecessor_orbit_complete"])
        self.assertNotIn(1, released)
        allowed, _ = completed_predecessor_handoff_allowed(
            trailing, middle, True, True, True, True, True, True, 3.0, 1.5)
        self.assertTrue(allowed)
        released.add(1)
        self.assertEqual([uid for uid in (3, 2, 1) if uid in released],
                         [3, 2, 1])

    def test_completed_predecessor_handoff_keeps_prediction_and_distance_gates(self):
        allowed, conditions = completed_predecessor_handoff_allowed(
            (0.0, 0.0, 28.0), (0.0, 0.0, 22.0),
            True, True, True, True, False, True, 3.0, 1.5)
        self.assertFalse(allowed)
        self.assertFalse(conditions["predicted_clear"])
        allowed, conditions = completed_predecessor_handoff_allowed(
            (0.0, 0.0, 23.0), (0.0, 0.0, 22.0),
            True, True, True, True, True, True, 3.0, 1.5)
        self.assertFalse(allowed)
        self.assertFalse(conditions["current_3d_clear"])
        self.assertFalse(conditions["current_ellipsoid_clear"])

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

    def test_leader_wait_keeps_follower_free_to_close_gap(self):
        positions = {
            3: self.point(40.0),
            2: self.point(305.0),  # 95 deg behind UAV3
            1: self.point(237.5),
        }
        holds, gaps, bands, reason = formation_phase_decision(
            positions, (-10.0, 20.0), [3, 2, 1], 1,
            57.5, 77.5, 45.0, 45.0, 95.0)
        self.assertEqual(reason, "OK")
        self.assertAlmostEqual(gaps["3-2"], 95.0)
        self.assertEqual(bands["3-2"], "LEADER_WAIT")
        self.assertEqual(holds, {3})

        # The predecessor remains stopped through hysteresis, but the follower
        # must remain permitted until the gap falls inside the normal band.
        positions[2] = self.point(306.0)
        holds, gaps, _, reason = formation_phase_decision(
            positions, (-10.0, 20.0), [3, 2, 1], 1,
            57.5, 77.5, 45.0, 45.0, 95.0, {3})
        self.assertEqual(reason, "OK")
        self.assertAlmostEqual(gaps["3-2"], 94.0)
        self.assertEqual(holds, {3})

        positions[2] = self.point(323.0)
        holds, gaps, _, reason = formation_phase_decision(
            positions, (-10.0, 20.0), [3, 2, 1], 1,
            57.5, 77.5, 45.0, 45.0, 95.0, {3})
        self.assertEqual(reason, "OK")
        self.assertAlmostEqual(gaps["3-2"], 77.0)
        self.assertNotIn(3, holds)

    def test_direct_hold_propagates_only_downstream_by_fixed_role(self):
        self.assertEqual(role_chain_hold_ids([3, 2, 1], 3), {2, 1})
        self.assertEqual(role_chain_hold_ids([3, 2, 1], 2), {1})
        self.assertEqual(role_chain_hold_ids([3, 2, 1], 1), set())
        self.assertEqual(role_chain_hold_ids([3, 2, 1], 99), set())

    def test_transition_never_bridges_over_missing_fixed_role(self):
        self.assertEqual(
            contiguous_role_segments([3, 2, 1], {2, 1}), [[2, 1]])
        self.assertEqual(
            contiguous_role_segments([3, 2, 1], {3, 1}), [[3], [1]])
        self.assertEqual(
            contiguous_role_segments([3, 2, 1], {3, 2}), [[3, 2]])
        self.assertEqual(
            contiguous_role_segments([3, 2, 1], {3, 2, 1}), [[3, 2, 1]])

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
            [3.0, 3.0, 3.0], 0.35, 0.2, True)
        self.assertFalse(ready)
        self.assertEqual(reasons[1], "READY")
        self.assertEqual(reasons[3],
                         "NOT_AT_ENTRY_GATE:ENTRY_GATE_TRANSIT")
        states[3] = state("ENTRY_READY")
        ready, reasons = entry_ready_barrier(
            [1, 2, 3], states, {1: True, 2: True, 3: True},
            [3.0, 3.0, 3.0], 0.35, 0.2, True)
        self.assertTrue(ready)

    def test_entry_ready_uses_each_uav_height(self):
        def state(height):
            return SimpleNamespace(
                mission_phase="ENTRY_READY", current_height=height,
                flight_state="HOVER_READY",
                velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0))
        states = {1: state(34.0), 2: state(28.0), 3: state(22.0)}
        ready, reasons = entry_ready_barrier(
            [1, 2, 3], states, {1: True, 2: True, 3: True},
            [34.0, 28.0, 22.0], 0.35, 0.2, True)
        self.assertTrue(ready)
        self.assertEqual(reasons, {1: "READY", 2: "READY", 3: "READY"})
        ready, reasons = entry_ready_barrier(
            [1, 2, 3], states, {1: True, 2: True, 3: True},
            [34.0, 28.0, 34.0], 0.35, 0.2, True)
        self.assertFalse(ready)
        self.assertEqual(reasons[3], "HEIGHT_NOT_READY")

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

    def test_three_uav_transition_owner_is_sticky_and_ordered(self):
        def state(phase, height, speed=0.0):
            return SimpleNamespace(
                mission_phase=phase, current_height=height,
                velocity=SimpleNamespace(x=speed, y=0.0, z=0.0))
        states = {
            3: state("WAIT_TRANSITION_PERMISSION", 22.0),
            2: state("WAIT_TRANSITION_PERMISSION", 28.0),
            1: state("WAIT_TRANSITION_PERMISSION", 34.0),
        }
        health = {1: True, 2: True, 3: True}
        targets = {1: 30.0, 2: 24.0, 3: 18.0}
        grants, owner, started, completed, _ = (
            serialized_transition_permissions(
                [3, 2, 1], states, health, targets, 0.35, 0.2, True))
        self.assertEqual(owner, 3)
        self.assertEqual(grants, {3: True, 2: False, 1: False})

        states[3] = state("LAYER_TRANSITION", 20.0)
        grants, owner, started, completed, _ = (
            serialized_transition_permissions(
                [3, 2, 1], states, health, targets, 0.35, 0.2, True,
                owner, started, completed))
        self.assertEqual(owner, 3)
        self.assertEqual(started, {3})
        self.assertEqual(grants, {3: True, 2: False, 1: False})

        grants, stale_owner, stale_started, stale_completed, reasons = (
            serialized_transition_permissions(
                [3, 2, 1], states, health, targets, 0.35, 0.2, False,
                owner, started, completed))
        self.assertEqual(stale_owner, 3)
        self.assertEqual(stale_started, {3})
        self.assertEqual(stale_completed, set())
        self.assertEqual(grants, {3: False, 2: False, 1: False})
        self.assertEqual(reasons[3], "STALE_OR_SAFETY_INVALID")

        states[3] = state("EVALUATING", 18.1)
        grants, owner, started, completed, _ = (
            serialized_transition_permissions(
                [3, 2, 1], states, health, targets, 0.35, 0.2, True,
                owner, started, completed))
        self.assertEqual(completed, {3})
        self.assertEqual(owner, 2)
        self.assertEqual(grants, {3: False, 2: True, 1: False})

        states[2] = state("LAYER_TRANSITION", 26.0)
        grants, owner, started, completed, _ = (
            serialized_transition_permissions(
                [3, 2, 1], states, health, targets, 0.35, 0.2, True,
                owner, started, completed))
        states[2] = state("TARGET_LOCKED", 24.0)
        grants, owner, started, completed, _ = (
            serialized_transition_permissions(
                [3, 2, 1], states, health, targets, 0.35, 0.2, True,
                owner, started, completed))
        self.assertEqual(completed, {3, 2})
        self.assertEqual(owner, 1)
        self.assertEqual(grants, {3: False, 2: False, 1: True})

    def test_transition_owner_requires_stable_target_and_fails_closed(self):
        def state(phase, height, speed=0.0):
            return SimpleNamespace(
                mission_phase=phase, current_height=height,
                velocity=SimpleNamespace(x=speed, y=0.0, z=0.0))
        states = {
            3: state("LAYER_TRANSITION", 20.0),
            2: state("WAIT_TRANSITION_PERMISSION", 28.0),
            1: state("WAIT_TRANSITION_PERMISSION", 34.0),
        }
        health = {1: True, 2: True, 3: True}
        targets = {1: 30.0, 2: 24.0, 3: 18.0}
        grants, owner, started, completed, _ = (
            serialized_transition_permissions(
                [3, 2, 1], states, health, targets, 0.35, 0.2, True,
                3, {3}, set()))
        states[3] = state("EVALUATING", 18.0, speed=0.3)
        grants, owner, started, completed, reasons = (
            serialized_transition_permissions(
                [3, 2, 1], states, health, targets, 0.35, 0.2, True,
                owner, started, completed))
        self.assertEqual(owner, 3)
        self.assertEqual(completed, set())
        self.assertEqual(reasons[3], "OWNER_AWAITING_STABLE_TARGET")
        self.assertFalse(any(grants.values()))
        health[2] = False
        grants, owner, _, _, reasons = serialized_transition_permissions(
            [3, 2, 1], states, health, targets, 0.35, 0.2, True,
            owner, started, completed)
        self.assertEqual(owner, 3)
        self.assertFalse(any(grants.values()))
        self.assertEqual(reasons[1], "STALE_OR_SAFETY_INVALID")

    def test_single_layer_transition_scheduler_is_disabled(self):
        states = {
            uid: SimpleNamespace(
                mission_phase="WAIT_EXIT_PERMISSION", current_height=3.0,
                velocity=SimpleNamespace(x=0.0, y=0.0, z=0.0))
            for uid in (1, 2, 3)}
        grants, owner, started, completed, reasons = (
            serialized_transition_permissions(
                [3, 2, 1], states, {uid: True for uid in states},
                {1: 3.0, 2: 3.0, 3: 3.0}, 0.35, 0.2, True,
                enabled=False))
        self.assertFalse(any(grants.values()))
        self.assertEqual(owner, 0)
        self.assertEqual(started, set())
        self.assertEqual(completed, set())
        self.assertEqual(reasons[3], "MULTI_LAYER_DISABLED")

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
