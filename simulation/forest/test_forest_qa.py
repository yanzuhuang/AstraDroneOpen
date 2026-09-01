#!/usr/bin/env python3

import json
from pathlib import Path
import unittest

from verify_forest_randomization_v1 import (
    generated_layout,
    load_config,
    parse_world,
    qualify_records,
)


class QualifiedForestPoolTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = Path(__file__).resolve().parent
        cls.config = load_config(cls.directory / "forest_pool_v1.json")

    def test_seed0_to_seed9_are_deterministic_unique_and_qualified(self):
        hashes = set()
        for item in self.config["seed_pool"]:
            logical_seed = int(item["logical_seed"])
            raw_seed = int(item["raw_seed"])
            first = generated_layout(raw_seed, self.config)
            second = generated_layout(raw_seed, self.config)
            self.assertEqual(first, second)
            world = parse_world(
                self.directory
                / "learning_speed_forest_v1_seed{}.world".format(logical_seed),
                self.config,
            )
            result = qualify_records(
                world, self.config, expected_raw_seed=raw_seed
            )
            self.assertTrue(result["passed"], result)
            self.assertEqual(
                result["counts"],
                {"sparse": 3, "medium": 6, "dense": 9, "total": 18},
            )
            self.assertEqual(result["overlap"]["overlap_count"], 0)
            self.assertTrue(result["traversability"]["passed"])
            self.assertNotIn(result["layout_sha256"], hashes)
            hashes.add(result["layout_sha256"])
        self.assertEqual(len(hashes), 10)

    def test_frozen_geometry_and_scheduler_config(self):
        self.assertEqual(self.config["map"]["size_x_m"], 60.0)
        self.assertEqual(self.config["map"]["size_y_m"], 20.0)
        self.assertEqual(self.config["map"]["start_safe_x_m"], [0.0, 10.0])
        self.assertEqual(self.config["map"]["end_safe_x_m"], [55.0, 60.0])
        self.assertEqual(self.config["cylinders"]["height_m"], 5.0)
        self.assertEqual(
            [self.config["zones"][name]["density_per_m2"] for name in ("sparse", "medium", "dense")],
            [0.01, 0.02, 0.03],
        )
        self.assertEqual(
            self.config["scheduler"]["training_logical_seeds"], list(range(8))
        )
        self.assertEqual(
            self.config["scheduler"]["evaluation_logical_seeds"], [8, 9]
        )


if __name__ == "__main__":
    unittest.main()
