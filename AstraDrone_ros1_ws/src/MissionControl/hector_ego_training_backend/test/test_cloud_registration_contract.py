#!/usr/bin/env python3

import math
import unittest

import numpy as np

from hector_ego_training_backend.cloud_registration_contract import (
    register_sensor_points_world,
    segment_corridor_statistics,
)


class CloudRegistrationContractTest(unittest.TestCase):
    def test_identity_pose_applies_verified_sensor_translation(self):
        result = register_sensor_points_world(
            [[1.0, 2.0, 3.0]], [10.0, 20.0, 30.0], [0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.13],
        )
        np.testing.assert_allclose(result, [[11.0, 22.0, 33.13]])

    def test_body_yaw_rotates_sensor_point_into_world(self):
        q = [0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)]
        result = register_sensor_points_world(
            [[1.0, 0.0, 0.0]], [0.0, 0.0, 0.0], q, [0.0, 0.0, 0.0]
        )
        np.testing.assert_allclose(result, [[0.0, 1.0, 0.0]], atol=1.0e-9)

    def test_corridor_counts_only_flight_band_obstacles(self):
        stats = segment_corridor_statistics(
            np.asarray([[1.0, 0.2, 3.0], [1.0, 2.0, 3.0], [1.0, 0.1, 0.0]]),
            [0.0, 0.0, 3.0], [2.0, 0.0, 3.0], 3.0, 0.5, 0.8,
        )
        self.assertEqual(stats["flight_band_points"], 2)
        self.assertEqual(stats["corridor_points"], 1)
        self.assertAlmostEqual(stats["minimum_distance"], 0.2)


if __name__ == "__main__":
    unittest.main()
