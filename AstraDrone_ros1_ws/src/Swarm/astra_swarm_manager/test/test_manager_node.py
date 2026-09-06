import importlib.util
import math
import os
import unittest
from types import SimpleNamespace


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(__file__)), "scripts",
    "swarm_manager_node.py")
SPEC = importlib.util.spec_from_file_location("swarm_manager_node", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class FakeDuration:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def to_sec(self):
        return self.seconds


class FakeTime:
    def __init__(self, seconds):
        self.seconds = float(seconds)

    def __sub__(self, other):
        return FakeDuration(self.seconds - other.seconds)

    def to_sec(self):
        return self.seconds


class RecordingPublisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(message.data)


def corridor(uid, angle_degrees, height, generation):
    angle = math.radians(angle_degrees)

    def point(radius):
        return (radius * math.cos(angle), radius * math.sin(angle), height)

    path = [point(18.0), point(15.0), point(12.5)]
    return {
        "id": "U{}_G{}".format(uid, generation),
        "angle_deg": angle_degrees,
        "pre_radius": 18.0,
        "entry_radius": 15.0,
        "orbit_staging_radius": 12.5,
        "radius_tier": 0,
        "endpoint_clearance": 1.0,
        "endpoint_valid": True,
        "path_valid": True,
        "ego_status": "NOT_PRECHECKED",
        "ego_candidate_valid": True,
        "clearance": 1.0,
        "pre": path[0],
        "entry": path[1],
        "staging": path[2],
        "path": path,
        "ingress_length": 5.5,
    }


def raw_message(candidate, generation, locked_candidate_id):
    item = SimpleNamespace(
        candidate_id=candidate["id"] + "/ORBIT_STAGING",
        accepted=True)
    return SimpleNamespace(
        current_sector=generation,
        locked_candidate_id=locked_candidate_id,
        candidates=[item])


class ManagerNodeGenerationReconciliationTest(unittest.TestCase):
    def test_g1_to_g2_revokes_suffix_republishes_and_rebuilds_uav1(self):
        manager = MODULE.SwarmManager.__new__(MODULE.SwarmManager)
        manager.uav_ids = [1, 2, 3]
        manager.role_order = [3, 2, 1]
        manager.phases = [292.5, 315.0, 337.5]
        manager.tower_center = [0.0, 0.0]
        manager.direction = 1
        manager.orbit_radius = 12.5
        manager.map_additional_clearance = 0.5
        manager.minimum_3d = 3.0
        manager.clearance = 1.5
        manager.timeout = 1.0
        manager.task_started = True
        manager.orbit_released = {3}
        manager.role_layers = {1:0,2:0,3:0}
        manager.released_layers = {1:-1,2:-1,3:0}

        leader = corridor(3, 337.5, 22.0, 1)
        middle_g1 = corridor(2, 315.0, 28.0, 1)
        middle_g2 = corridor(2, 315.0, 28.0, 2)
        trailing = corridor(1, 292.5, 34.0, 1)
        candidates = {3: leader, 2: middle_g2, 1: trailing}
        manager.entry_corridor_selection = {
            3: leader, 2: middle_g1, 1: trailing}
        manager.entry_corridor_commit_state = {
            uid: {
                "selected_candidate_id": manager.entry_corridor_selection[uid]["id"],
                "candidate_generation": 1,
                "committed": True,
                "entered": uid == 3,
                "released": uid == 3,
                "mission_lock_observed": True,
            }
            for uid in manager.role_order
        }
        manager.entry_corridor_generation_key = ((3, 1), (2, 1), (1, 1))
        manager.entry_corridor_selection_reason = "ALL_ROLES_COMMITTED"
        manager.entry_corridor_diagnostics = {}
        manager.entry_corridor_audit = {}
        manager.entry_corridor_selection_pubs = {
            uid: RecordingPublisher() for uid in manager.uav_ids}
        manager.entry_corridor_audit_pub = RecordingPublisher()

        now = FakeTime(10.0)
        manager.entry_corridor_received = {
            uid: now for uid in manager.uav_ids}
        manager.entry_corridor_messages = {
            3: raw_message(leader, 1, leader["id"]),
            2: raw_message(middle_g2, 2, ""),
            1: raw_message(trailing, 1, trailing["id"]),
        }
        manager.states = {
            uid: SimpleNamespace(
                mission_phase=("ENTRY_READY" if uid == 3
                               else "WAIT_ENTRY_PERMISSION"),
                pose=SimpleNamespace(position=SimpleNamespace(
                    x=candidates[uid]["pre"][0],
                    y=candidates[uid]["pre"][1],
                    z=candidates[uid]["pre"][2])))
            for uid in manager.uav_ids
        }
        manager.predictions = {
            uid: SimpleNamespace(points=[SimpleNamespace(x=x, y=y, z=z)
                                        for x, y, z in candidates[uid]["path"]])
            for uid in manager.uav_ids
        }
        manager.healthy = lambda uid, stamp: True
        manager.entry_corridor_candidates = lambda uid: [candidates[uid]]

        grants = manager.update_entry_corridor_selection(now, True)
        self.assertEqual(set(manager.entry_corridor_selection), {3, 2})
        self.assertEqual(
            manager.entry_corridor_selection[2]["id"], middle_g2["id"])
        self.assertEqual(grants, {3: True, 2: True, 1: False})
        self.assertEqual(
            manager.entry_corridor_selection_pubs[2].values,
            ["", middle_g2["id"]])
        self.assertEqual(
            manager.entry_corridor_selection_pubs[1].values, [""])
        self.assertTrue(manager.entry_corridor_commit_state[3]["entered"])
        self.assertTrue(manager.entry_corridor_commit_state[3]["released"])
        self.assertEqual(
            manager.entry_corridor_commit_state[2]["candidate_generation"],
            2)

        # The next incremental cycle rebuilds the downstream UAV1 selection;
        # role order remains UAV3 -> UAV2 -> UAV1.
        grants = manager.update_entry_corridor_selection(now, True)
        self.assertEqual(set(manager.entry_corridor_selection), {3, 2, 1})
        self.assertEqual(grants, {3: True, 2: True, 1: True})
        self.assertEqual(
            manager.entry_corridor_selection_pubs[1].values,
            ["", trailing["id"]])


class ManagerNodeMultiLayerCycleTest(unittest.TestCase):
    @staticmethod
    def manager(layer_offsets):
        manager = MODULE.SwarmManager.__new__(MODULE.SwarmManager)
        manager.uav_ids = [1, 2, 3]
        manager.heights = [26.0, 20.0, 14.0]
        manager.layer_offsets = list(layer_offsets)
        manager.active_layer_index = 0
        manager.transition_target_layer = 0
        manager.transition_target_heights = {1: 26.0, 2: 20.0, 3: 14.0}
        manager.transition_owner = 0
        manager.transition_started = set()
        manager.transition_completed = set()
        manager.transition_round_active = False
        manager.orbit_released = {1, 2, 3}
        manager.released_layers = {uid: 0 for uid in manager.uav_ids}
        manager.confirmed_release_layers = dict(manager.released_layers)
        manager.late_release_references = {}
        manager.orbit_release_times = {
            3: FakeTime(1.0), 2: FakeTime(2.0), 1: FakeTime(3.0)}
        manager.orbit_completed = {1, 2, 3}
        manager.formation_orbit_active = True
        manager.orbit_started = True
        manager.orbit_start_time = FakeTime(1.0)
        manager.release_diagnostics = {"3-2": {"released": True}}
        manager.phase_held = {1}
        manager.phase_gaps = {"3-2": 67.5}
        manager.phase_bands = {"uav1": "HOLD"}
        return manager

    def test_completion_resets_only_owner_and_immediate_second_orbit_release(self):
        manager = self.manager([0., -4.])
        manager.role_order = [3, 2, 1]
        manager.role_layers = {u: 0 for u in manager.uav_ids}
        manager.released_layers = dict(manager.role_layers)
        manager.confirmed_release_layers = dict(manager.role_layers)
        manager.transition_owner_started = False
        manager.transition_height_tolerance = .35
        manager.transition_maximum_speed = .2
        manager.entry_corridor_commit_state = {}
        manager.states = {u: SimpleNamespace(
            mission_phase="NAVIGATING", current_height=h,
            velocity=SimpleNamespace(x=0., y=0., z=0.))
            for u,h in zip(manager.uav_ids, manager.heights)}
        health = {u: True for u in manager.uav_ids}
        manager.states[3].mission_phase = "WAIT_TRANSITION_PERMISSION"
        self.assertTrue(manager.update_async_transitions(health, True)[3])
        self.assertEqual(manager.orbit_released, {1,2,3})
        manager.states[3].mission_phase = "LAYER_TRANSITION"
        manager.update_async_transitions(health, True)
        manager.states[3].mission_phase = "ORBIT_STAGING_READY"
        manager.states[3].current_height = 10.
        manager.update_async_transitions(health, True)
        self.assertEqual(manager.orbit_released, {1,2})
        self.assertEqual(manager.role_layers, {1:0,2:0,3:1})
        manager.release_orbit(3, FakeTime(10.), "own staging ready")
        self.assertEqual(manager.released_layers, {1:0,2:0,3:1})
        self.assertEqual(manager.orbit_release_times[2].to_sec(), 2.)
        self.assertEqual(manager.states[1].mission_phase, "NAVIGATING")
        self.assertEqual(manager.states[2].mission_phase, "NAVIGATING")

    def test_each_new_layer_follower_uses_moving_phase_release_without_land(self):
        for leader, follower, untouched in [(3,2,1),(2,1,3)]:
            manager = self.manager([0.,-4.])
            manager.role_order = [3,2,1]
            manager.role_layers = {1:0,2:1,3:1}
            manager.role_layers[follower] = 1
            manager.released_layers = {u:0 for u in manager.uav_ids}
            manager.confirmed_release_layers = dict(manager.released_layers)
            manager.released_layers[leader] = 1
            manager.orbit_released = {leader}
            manager.entry_corridor_commit_state = {}
            manager.tower_center = [0.,0.]
            manager.direction = 1
            manager.release_phase_min = 65.
            manager.release_phase_max = 70.
            manager.release_minimum_forward_speed = .05
            manager.healthy = lambda uid, now: True
            manager.pair_prediction_clear = lambda first, second: True
            manager.orbit_staging_ready_reasons = {follower:"READY"}
            a = math.radians(67.5)
            manager.states = {
                leader: SimpleNamespace(mission_phase="NAVIGATING", flight_state="TRACK_EGO",
                    pose=SimpleNamespace(position=SimpleNamespace(x=12.5*math.cos(a),y=12.5*math.sin(a),z=18.)),
                    velocity=SimpleNamespace(x=-.6*math.sin(a),y=.6*math.cos(a),z=0.)),
                follower: SimpleNamespace(mission_phase="ORBIT_STAGING_READY",flight_state="HOVER_READY",
                    pose=SimpleNamespace(position=SimpleNamespace(x=12.5,y=0.,z=24.)),
                    velocity=SimpleNamespace(x=0.,y=0.,z=0.))}
            allowed, phase, speed, conditions = manager.evaluate_sequential_release(
                leader,follower,FakeTime(20.),True)
            self.assertTrue(allowed, conditions)
            self.assertAlmostEqual(phase,67.5)
            self.assertGreater(speed,0.)
            manager.release_orbit(follower,FakeTime(20.),"PHASE_WINDOW")
            self.assertEqual(manager.released_layers[follower],1)
            self.assertEqual(manager.states[leader].mission_phase,"NAVIGATING")
            manager.role_layers[follower]=0
            self.assertFalse(manager.evaluate_sequential_release(leader,follower,FakeTime(21.),True)[0])

    def test_target_layer_late_follower_is_immediately_safe_to_release(self):
        manager = self.manager([0., -4.])
        manager.role_order = [3, 2, 1]
        manager.role_layers = {3: 1, 2: 1, 1: 0}
        manager.orbit_released = {3}
        manager.tower_center = [0., 0.]
        manager.direction = 1
        manager.release_phase_min = 65.
        manager.release_phase_max = 70.
        manager.release_minimum_forward_speed = .05
        manager.entry_corridor_commit_state = {}
        manager.healthy = lambda uid, now: True
        manager.pair_prediction_clear = lambda first, second: True
        manager.orbit_staging_ready_reasons = {2: "READY"}
        manager.states = {
            3: SimpleNamespace(
                mission_phase="NAVIGATING", flight_state="TRACK_EGO",
                pose=SimpleNamespace(position=SimpleNamespace(
                    x=12.5 * math.cos(math.radians(86.6)),
                    y=12.5 * math.sin(math.radians(86.6)), z=14.)),
                velocity=SimpleNamespace(
                    x=-.6 * math.sin(math.radians(86.6)),
                    y=.6 * math.cos(math.radians(86.6)), z=0.)),
            2: SimpleNamespace(
                mission_phase="ORBIT_STAGING_READY", flight_state="HOVER_READY",
                pose=SimpleNamespace(position=SimpleNamespace(
                    x=12.5, y=0., z=20.)),
                velocity=SimpleNamespace(x=0., y=0., z=0.)),
        }
        allowed, phase, _, conditions = manager.evaluate_sequential_release(
            3, 2, FakeTime(20.), True)
        self.assertFalse(allowed)
        self.assertAlmostEqual(phase, 86.6)
        self.assertTrue(MODULE.target_layer_late_release_allowed(
            phase, manager.release_phase_max, conditions))
        blocked_conditions = dict(conditions)
        blocked_conditions["predicted_clear"] = False
        self.assertFalse(MODULE.target_layer_late_release_allowed(
            phase, manager.release_phase_max, blocked_conditions))

    def test_initial_layer_late_follower_keeps_original_phase_window(self):
        manager = self.manager([0., -4.])
        manager.role_order = [3, 2, 1]
        manager.role_layers = {3: 0, 2: 0, 1: 0}
        manager.tower_center = [0., 0.]
        manager.direction = 1
        manager.release_phase_min = 65.
        manager.release_phase_max = 70.
        manager.release_minimum_forward_speed = .05
        manager.healthy = lambda uid, now: True
        manager.pair_prediction_clear = lambda first, second: True
        manager.orbit_staging_ready_reasons = {2: "READY"}
        manager.states = {
            3: SimpleNamespace(
                mission_phase="NAVIGATING", flight_state="TRACK_EGO",
                pose=SimpleNamespace(position=SimpleNamespace(
                    x=12.5 * math.cos(math.radians(75.)),
                    y=12.5 * math.sin(math.radians(75.)), z=14.)),
                velocity=SimpleNamespace(
                    x=-.6 * math.sin(math.radians(75.)),
                    y=.6 * math.cos(math.radians(75.)), z=0.)),
            2: SimpleNamespace(
                mission_phase="ORBIT_STAGING_READY", flight_state="HOVER_READY",
                pose=SimpleNamespace(position=SimpleNamespace(
                    x=12.5, y=0., z=20.)),
                velocity=SimpleNamespace(x=0., y=0., z=0.)),
        }
        allowed, phase, _, conditions = manager.evaluate_sequential_release(
            3, 2, FakeTime(20.), True)
        self.assertFalse(allowed)
        reference = manager.update_late_release_reference(
            3, 2, phase, conditions)
        self.assertIsNotNone(reference)
        angle = math.radians(142.5)
        manager.states[3].pose.position.x = 12.5 * math.cos(angle)
        manager.states[3].pose.position.y = 12.5 * math.sin(angle)
        manager.states[3].velocity.x = -.6 * math.sin(angle)
        manager.states[3].velocity.y = .6 * math.cos(angle)
        allowed, progress, _, late_conditions = (
            manager.evaluate_late_follower_release(
                3, 2, reference, conditions))
        self.assertTrue(allowed, late_conditions)
        self.assertAlmostEqual(progress, 67.5)

    def test_handoff_waits_until_follower_consumes_its_release(self):
        manager = self.manager([0.,-4.])
        manager.role_order = [3,2,1]
        manager.orbit_released = {3,2}
        manager.released_layers = {3:1,2:1,1:0}
        manager.confirmed_release_layers = {3:1,2:0,1:0}
        manager.states = {3:SimpleNamespace(mission_phase="NAVIGATING"),
                          2:SimpleNamespace(mission_phase="ORBIT_STAGING_READY")}
        health = {1:True,2:True,3:True}
        manager.confirm_orbit_releases(health)
        self.assertEqual(manager.confirmed_release_layers[2],0)
        manager.states[2].mission_phase = "NAVIGATING"
        health[2] = False
        manager.confirm_orbit_releases(health)
        self.assertEqual(manager.confirmed_release_layers[2],0)
        health[2] = True
        manager.confirm_orbit_releases(health)
        self.assertEqual(manager.confirmed_release_layers[2],1)




if __name__ == "__main__":
    unittest.main()
