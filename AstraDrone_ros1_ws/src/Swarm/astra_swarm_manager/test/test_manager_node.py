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


if __name__ == "__main__":
    unittest.main()
