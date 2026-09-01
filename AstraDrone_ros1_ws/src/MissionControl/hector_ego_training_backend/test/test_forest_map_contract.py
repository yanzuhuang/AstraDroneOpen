#!/usr/bin/env python3

import json
from pathlib import Path
import unittest

from hector_ego_training_backend.forest_map_contract import ForestPoolContract


class ForestMapContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = Path(__file__).resolve().parents[5]
        cls.forest = cls.repo / "simulation/forest"
        cls.contract = ForestPoolContract(
            cls.forest / "forest_pool_v1.json", cls.forest
        )

    def test_world_pool_has_ten_qualified_18_obstacle_layouts(self):
        self.assertEqual(set(self.contract.layouts), set(range(10)))
        self.assertTrue(all(len(layout) == 18 for layout in self.contract.layouts.values()))
        self.assertEqual(len({self.contract.layout_hash(seed) for seed in range(10)}), 10)

    def test_training_and_evaluation_requests_are_isolated(self):
        mapping = self.contract.mapping()
        training = {
            "request_id": "run:block:0", "logical_seed": 3,
            "raw_seed": mapping[3], "mode": "training",
            "map_block_id": 0, "map_round_id": 0,
            "scheduler_order": [3, 7, 1, 5, 0, 6, 2, 4],
        }
        self.assertEqual(self.contract.validate_request(training)["logical_seed"], 3)
        invalid = dict(training, logical_seed=8, raw_seed=mapping[8])
        with self.assertRaises(ValueError):
            self.contract.validate_request(invalid)
        evaluation = {
            "request_id": "eval:8", "logical_seed": 8,
            "raw_seed": mapping[8], "mode": "evaluation",
            "map_block_id": -1, "map_round_id": -1,
            "scheduler_order": [8],
        }
        self.assertEqual(self.contract.validate_request(evaluation)["mode"], "evaluation")

    def test_loaded_layout_rejects_residual_or_stale_pose(self):
        expected = self.contract.layouts[0]
        positions = {item.name: (item.x, item.y, item.z) for item in expected}
        names = set(positions) | {"hector_uav1", "forest_ground_60x20"}
        self.assertTrue(self.contract.verify_loaded_layout(expected, names, positions)["passed"])
        residual_names = set(names) | {"dense_99"}
        self.assertFalse(self.contract.verify_loaded_layout(expected, residual_names, positions)["passed"])
        positions[expected[0].name] = (expected[0].x + 1.0, expected[0].y, expected[0].z)
        self.assertFalse(self.contract.verify_loaded_layout(expected, names, positions)["passed"])


if __name__ == "__main__":
    unittest.main()
