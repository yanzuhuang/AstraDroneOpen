#!/usr/bin/env python3
"""Launch contract regressions; loads configuration only, starts no ROS nodes."""
import copy
import importlib.util
import os
from pathlib import Path
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('height_contract', PACKAGE / 'scripts/check_height_profile.py')
contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(contract)


class HeightProfilesTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        repository = PACKAGE.parents[3]
        px4 = repository.parent / 'PX4-Autopilot'
        os.environ['ROS_PACKAGE_PATH'] = os.pathsep.join([
            str(repository / 'simulation/sim_workspace/src'), str(px4),
            str(px4 / 'Tools/simulation/gazebo-classic/sitl_gazebo-classic'),
            os.environ.get('ROS_PACKAGE_PATH', '')])
        cls.profiles = {
            name: contract.expand('triple_tower_height_profile.launch', ['height_profile:=' + name])
            for name in ('same_3m', 'same_30m', 'multi_height_low_3m')}

    def test_all_profiles_and_explicit_expected_heights(self):
        for name, expected in [('same_3m', [3, 3, 3]), ('same_30m', [30, 30, 30]),
                               ('multi_height_low_3m', [15, 9, 3])]:
            rows = contract.validate(self.profiles[name])
            self.assertEqual([r['top'] for r in rows], expected)
            self.assertEqual([r['layers'] for r in rows], [[h] for h in expected])
            self.assertEqual([r['mission_range'][1] for r in rows], [h + 1.5 for h in expected])
        rows = contract.validate(self.profiles['multi_height_low_3m'])
        self.assertEqual([r['takeoff'] for r in rows], [4, 4, 2.5])
        self.assertEqual([r['staging'] for r in rows], [4, 4, 2.5])
        self.assertEqual([r['ascent'] for r in rows], [[10], [], []])

    def test_only_height_parameters_change_from_existing_launches(self):
        # Exhaustive comparison, not a sample of speed/safety parameters.
        allowed = ('mission/inspection_top_height', 'mission/layer_offsets',
                   'fsm/manual_target_height', 'mission/transit_height',
                   'staging/height', 'staging/ascent_intermediate_heights',
                   'mission/maximum_height', 'mission/virtual_ceil_height',
                   'mission/recovery_height_max', 'recovery/recovery_height',
                   'return_egress/transit_height', 'low_altitude/entry_height',
                   'grid_map/virtual_ceil_height', 'takeoff_height', 'return_height',
                   'max_relative_height', 'takeoff_timeout', 'virtual_ceil_height',
                   'mission_heights', 'takeoff_heights', 'uav2_takeoff_height')
        for name, new in self.profiles.items():
            launch = ('triple_tower_multi_height_inspection.launch' if name.startswith('multi')
                      else 'triple_tower_inspection.launch')
            old = contract.expand(launch, [])
            differences = {key for key in set(old) | set(new) if old.get(key) != new.get(key)}
            unexpected = [key for key in differences if not any(key.endswith('/' + s) for s in allowed)]
            self.assertEqual(unexpected, [], (name, unexpected))

    def test_custom_height_preserves_named_profile_parameters(self):
        for height, name in [(3, 'same_3m'), (30, 'same_30m')]:
            values = contract.expand('triple_tower_height_profile.launch',
                                     ['height_profile:=same_custom', 'same_altitude:=' + str(height)])
            self.assertEqual(values, self.profiles[name])

    def test_rejects_top_only_change(self):
        values = copy.deepcopy(self.profiles['same_3m'])
        values['/uav1/tower_mission/mission/inspection_top_height'] = 30.0
        with self.assertRaises(ValueError):
            contract.validate(values)

    def test_rejects_low_first_layer_with_old_staging_and_ascent(self):
        for key, value in [('staging/height', 4.0),
                           ('staging/ascent_intermediate_heights', [10.0, 18.0]),
                           ('mission/layer_offsets', [0.0, -4.0])]:
            values = copy.deepcopy(self.profiles['multi_height_low_3m'])
            values['/uav3/tower_mission/' + key] = value
            with self.assertRaises(ValueError):
                contract.validate(values)

    def test_rejects_manager_mismatch_and_30m_short_timeout(self):
        for key, value in [('/astra_swarm_manager/mission_heights', [3, 3, 3]),
                           ('/uav1/ego_mavros_bridge/takeoff_timeout', 60.0)]:
            values = copy.deepcopy(self.profiles['same_30m'])
            values[key] = value
            with self.assertRaises(ValueError):
                contract.validate(values)

    def test_unknown_profile_rejected(self):
        with self.assertRaises(Exception):
            contract.expand('triple_tower_height_profile.launch', ['height_profile:=typo'])


if __name__ == '__main__':
    unittest.main()
