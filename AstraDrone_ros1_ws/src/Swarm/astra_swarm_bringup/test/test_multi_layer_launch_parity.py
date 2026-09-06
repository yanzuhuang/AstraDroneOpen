#!/usr/bin/env python3

import collections
import os
from pathlib import Path
import shlex
import unittest
import xml.etree.ElementTree as ET


class MultiLayerLaunchParityTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = Path(__file__).resolve().parents[1]
        cls.repository = Path(__file__).resolve().parents[5]
        cls.px4 = cls.repository.parent / "PX4-Autopilot"
        cls.simulation_sources = (
            cls.repository / "simulation" / "sim_workspace" / "src")

        required_paths = [cls.px4, cls.simulation_sources]
        missing = [str(path) for path in required_paths if not path.exists()]
        if missing:
            raise AssertionError(
                "required launch dependency paths are missing: {}".format(
                    missing))
        px4_gazebo = (
            cls.px4 / "Tools" / "simulation" / "gazebo-classic" /
            "sitl_gazebo-classic")
        current_ros_path = os.environ.get("ROS_PACKAGE_PATH", "")
        os.environ["ROS_PACKAGE_PATH"] = os.pathsep.join(
            [str(cls.px4), str(px4_gazebo), str(cls.simulation_sources),
             current_ros_path])

        import roslaunch

        common = [
            "enable_control:=false",
            "start_sim:=true",
            "gui:=false",
            "start_rviz:=false",
            "start_rviz_diagnostics:=false",
            "d435_enabled:=true",
            "lidar_downsample:=1",
        ]
        cls.single = roslaunch.config.load_config_default(
            [(str(cls.package / "launch" /
                  "triple_tower_inspection.launch"), common)],
            None, assign_machines=False)
        cls.multi = roslaunch.config.load_config_default(
            [(str(cls.package / "launch" /
                  "triple_tower_multi_height_inspection.launch"),
              ["multi_layer_enabled:=true"] + common)],
            None, assign_machines=False)

    @staticmethod
    def _node_inventory(config):
        return collections.Counter(
            (node.package, node.type, node.namespace, node.name)
            for node in config.nodes)

    @staticmethod
    def _gazebo_world(config):
        servers = [
            node for node in config.nodes
            if node.package == "gazebo_ros" and node.type == "gzserver"]
        if len(servers) != 1:
            raise AssertionError(
                "expected exactly one gzserver, got {}".format(len(servers)))
        worlds = [
            token for token in shlex.split(servers[0].args)
            if token.endswith(".world")]
        if len(worlds) != 1:
            raise AssertionError(
                "expected exactly one world argument, got {}".format(worlds))
        return Path(worlds[0]).resolve()

    @staticmethod
    def _spawn_inventory(config):
        return collections.Counter(
            (node.package, node.type, node.namespace)
            for node in config.nodes if "spawn" in node.type)

    @staticmethod
    def _sensor_payload_counts(config):
        counts = {}
        for name, parameter in config.params.items():
            if "/sdf_iris_mid360_" not in name:
                continue
            root = ET.fromstring(parameter.value)
            counts[name.split("/")[1]] = (
                len(root.findall(".//sensor")),
                len(root.findall(".//plugin")),
            )
        return counts

    def test_single_and_multi_layer_have_identical_gazebo_wiring(self):
        self.assertEqual(
            self._gazebo_world(self.single), self._gazebo_world(self.multi))
        self.assertEqual(
            self._node_inventory(self.single),
            self._node_inventory(self.multi))
        self.assertEqual(
            self._spawn_inventory(self.single),
            self._spawn_inventory(self.multi))
        self.assertEqual(sum(self._spawn_inventory(self.multi).values()), 3)
        self.assertEqual(
            self._sensor_payload_counts(self.single),
            self._sensor_payload_counts(self.multi))
        self.assertEqual(
            self._sensor_payload_counts(self.multi),
            {"uav1": (7, 4), "uav2": (7, 4), "uav3": (7, 4)})

    def test_same_height_speed_chain_matches_multi_height_profile(self):
        single = {name: parameter.value
                  for name, parameter in self.single.params.items()}
        multi = {name: parameter.value
                 for name, parameter in self.multi.params.items()}
        self.assertEqual(single["/astra_swarm_manager/entry_nominal_speed"],
                         0.60)
        self.assertEqual(single["/astra_swarm_manager/orbit_nominal_speed"],
                         0.60)
        for uid, planner_id in ((1, 0), (2, 1), (3, 2)):
            planner = "/uav{}/drone_{}_ego_planner_node/".format(
                uid, planner_id)
            bridge = "/uav{}/ego_mavros_bridge/".format(uid)
            for suffix in ("manager/max_vel", "optimization/max_vel",
                           "bspline/limit_vel"):
                self.assertEqual(single[planner + suffix], 0.60)
                self.assertEqual(single[planner + suffix],
                                 multi[planner + suffix])
            self.assertEqual(single[bridge + "max_velocity"], 0.60)
            self.assertEqual(single[bridge + "max_acceleration"], 0.50)
            self.assertTrue(single[
                bridge + "scale_command_limits_with_orbit_speed_scale"])
            for suffix in ("max_velocity", "max_acceleration",
                           "scale_command_limits_with_orbit_speed_scale"):
                self.assertEqual(single[bridge + suffix],
                                 multi[bridge + suffix])

    def test_shared_worksite_has_no_duplicate_top_level_models(self):
        world_path = self._gazebo_world(self.multi)
        expected = (
            self.repository / "simulation" / "astra_gazebo_worlds" /
            "worksite.world").resolve()
        self.assertEqual(world_path, expected)
        world = ET.parse(str(world_path)).getroot().find("world")
        self.assertIsNotNone(world)
        self.assertEqual(world.get("name"), "default")
        names = [model.get("name") for model in world.findall("model")]
        self.assertEqual(len(names), 42)
        self.assertEqual(len(names), len(set(names)))
        self.assertEqual(len(world.findall("include")), 0)
        self.assertEqual(len(world.findall("plugin")), 0)
        physics = world.find("physics")
        self.assertEqual(physics.findtext("max_step_size"), "0.001")
        self.assertEqual(physics.findtext("real_time_update_rate"), "1000")

    def test_multi_height_profile_resolves_requested_layers_and_role_order(self):
        values = {name: parameter.value
                  for name, parameter in self.multi.params.items()}
        self.assertEqual(
            [values["/uav{}/tower_mission/mission/inspection_top_height".format(uid)]
             for uid in (1, 2, 3)],
            [26.0, 20.0, 14.0])
        self.assertEqual(
            [values["/uav{}/tower_mission/mission/layer_offsets".format(uid)]
             for uid in (1, 2, 3)],
            [[0.0, -4.0], [0.0, -4.0], [0.0, -4.0]])
        self.assertEqual(
            [[top + offset for offset in offsets]
             for top, offsets in zip(
                 [26.0, 20.0, 14.0],
                 [[0.0, -4.0], [0.0, -4.0], [0.0, -4.0]])],
            [[26.0, 22.0], [20.0, 16.0], [14.0, 10.0]])
        self.assertEqual(
            [values["/uav{}/tower_mission/staging/ascent_intermediate_heights".format(uid)]
             for uid in (1, 2, 3)],
            [[10.0, 18.0], [10.0, 18.0], [10.0]])
        for uid in (1, 2, 3):
            prefix = "/uav{}/tower_mission/candidate/".format(uid)
            self.assertTrue(values[prefix + "high_altitude_policy"])
            self.assertFalse(
                values[prefix + "known_obstacle_is_hard_constraint"])
            # Historical single-UAV local-descent candidates must not leak
            # into a formal multi-layer orbit.  Every ordinary candidate is
            # generated exactly at its layer target height.
            self.assertEqual(values[prefix + "height_offsets_m"], [0.0])
            self.assertEqual(
                values[prefix + "radius_offsets_m"], [0.0, 2.0, 4.0])
        self.assertEqual(
            values["/astra_swarm_manager/transition_order"], [3, 2, 1])
        self.assertEqual(
            values["/astra_swarm_manager/layer_offsets"], [0.0, -4.0])


if __name__ == "__main__":
    unittest.main()
