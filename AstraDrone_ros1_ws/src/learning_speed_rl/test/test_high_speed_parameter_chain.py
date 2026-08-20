#!/usr/bin/env python3

import importlib.util
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[4]
SCRIPT = REPO_ROOT / (
    "AstraDrone_ros1_ws/src/learning_speed_rl/scripts/"
    "high_speed_parameter_preflight.py")
SPEC = importlib.util.spec_from_file_location("high_speed_parameter_preflight", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class HighSpeedParameterChainTest(unittest.TestCase):
    def test_reviewed_chain_accepts_every_qualification_request(self):
        result = MODULE.run_preflight(
            REPO_ROOT,
            MODULE.QUALIFICATION_SPEEDS,
            MODULE.EXPECTED_CEILING,
            MODULE.EXPECTED_MAX_ACC,
            MODULE.EXPECTED_HORIZON,
            REPO_ROOT / (
                "AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/config/"
                "uav1_high_speed_bridge.yaml"),
        )
        self.assertTrue(result["pass"], result["checks"])
        self.assertFalse(result["flight_or_simulation_started"])

    def test_bridge_has_norm_headroom_for_ego_per_axis_limits(self):
        result = MODULE.run_preflight(
            REPO_ROOT, [3.5], 4.0, 3.0, 7.5,
            REPO_ROOT / (
                "AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/config/"
                "uav1_high_speed_bridge.yaml"),
        )
        checks = {item["name"]: item for item in result["checks"]}
        self.assertTrue(checks["bridge_velocity_norm_covers_ego_xyz_box"]["pass"])
        self.assertTrue(checks["bridge_acceleration_norm_covers_ego_xyz_box"]["pass"])

    def test_horizon_covers_reviewed_kinematic_budget(self):
        result = MODULE.run_preflight(
            REPO_ROOT, [3.5], 4.0, 3.0, 7.5,
            REPO_ROOT / (
                "AstraDrone_ros1_ws/src/Swarm/astra_swarm_bringup/config/"
                "uav1_high_speed_bridge.yaml"),
        )
        self.assertLess(
            result["required_horizon_at_ceiling_m"]["reaction_0p3_s"], 7.5)


if __name__ == "__main__":
    unittest.main()
