#!/usr/bin/env python3

import unittest

import numpy as np

from learning_speed_rl.observation.builder import (
    LowDimObservationBuilder,
    VehiclePlanningState,
)
from learning_speed_rl.observation.types import LOW_DIM_FIELDS, MapTensorSpec
from learning_speed_rl.observation.voxelizer import OccupiedPointCloudVoxelizer


class ObservationTest(unittest.TestCase):
    def test_occupied_pointcloud_does_not_invent_free_space(self):
        spec = MapTensorSpec((4.0, 4.0, 2.0), 1.0)
        tensor = OccupiedPointCloudVoxelizer(spec).build(
            occupied_points_xyz=[(0.1, 0.1, 0.1)],
            center_xyz=(0.0, 0.0, 0.0),
            trajectory_points_xyz=[(1.1, 0.1, 0.1)],
        )
        self.assertEqual(tensor.shape, (4, 2, 4, 4))
        self.assertEqual(int(np.count_nonzero(tensor[0])), 0)
        self.assertEqual(int(np.count_nonzero(tensor[1])), 1)
        self.assertEqual(int(np.count_nonzero(tensor[3])), 1)
        occupied_index = np.argwhere(tensor[1] > 0.5)[0]
        self.assertEqual(tensor[(2,) + tuple(occupied_index)], 0.0)

    def test_low_dim_contract_and_readiness_are_explicit(self):
        spec = MapTensorSpec((4.0, 4.0, 2.0), 1.0)
        state = VehiclePlanningState(
            stamp_sec=1.0,
            frame_id="camera_init",
            position=np.asarray([1.0, 2.0, 3.0], dtype=np.float32),
            desired_position=np.asarray([0.5, 1.0, 2.5], dtype=np.float32),
            local_goal=np.asarray([3.0, 2.0, 3.0], dtype=np.float32),
            have_command=True,
            have_goal=True,
        )
        observation = LowDimObservationBuilder().build(
            state,
            previous_v_max=0.4,
            map_spec=spec,
            map_tensor=np.zeros(spec.tensor_shape, dtype=np.float32),
        )
        self.assertEqual(observation.low_dim.shape, (len(LOW_DIM_FIELDS),))
        np.testing.assert_allclose(observation.low_dim[9:12], [0.5, 1.0, 0.5])
        np.testing.assert_allclose(observation.low_dim[12:15], [2.0, 0.0, 0.0])
        self.assertFalse(observation.ready_for_rl)


if __name__ == "__main__":
    unittest.main()
