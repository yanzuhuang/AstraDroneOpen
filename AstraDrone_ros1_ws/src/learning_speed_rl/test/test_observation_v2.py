#!/usr/bin/env python3

import math
import unittest

import numpy as np

from learning_speed_rl.observation.v2 import (
    AngularPartition,
    AngularPartitionSpec,
    BinSemantic,
    CloudFrame,
    CloudHistoryBuffer,
    FovCoverageConfig,
    HistoricalFovEstimator,
    LidarSurrogateBuilder,
    LidarSurrogateConfig,
    Pose3D,
    PoseBuffer,
    preserve_source_stamp,
    sensor_to_body,
)


def yaw_quaternion(degrees):
    half = math.radians(degrees) * 0.5
    return np.asarray([0.0, 0.0, math.sin(half), math.cos(half)])


def pose(stamp, xyz=(0.0, 0.0, 0.0), yaw_deg=0.0):
    return Pose3D(stamp, np.asarray(xyz), yaw_quaternion(yaw_deg))


def reference_partition():
    return AngularPartition(
        AngularPartitionSpec(
            angular_resolution_deg=4.5,
            expected_number_of_bins=3200,
        )
    )


def reference_builder(minimum_history_frames=1):
    partition = reference_partition()
    fov = HistoricalFovEstimator(
        FovCoverageConfig(
            sensor_min_range_m=0.2,
            distance_clip_m=10.0,
            radial_sample_step_m=0.25,
            azimuth_min_deg=-180.0,
            azimuth_max_deg=180.0,
            elevation_min_deg=-7.2123,
            elevation_max_deg=52.164,
            occlusion_angular_resolution_deg=4.5,
            occlusion_margin_m=0.10,
            sensor_translation_body_m=(0.0, 0.0, 0.0),
            sensor_rotation_body_xyzw=(0.0, 0.0, 0.0, 1.0),
        )
    )
    return LidarSurrogateBuilder(
        LidarSurrogateConfig(
            voxel_downsample_m=0.05,
            sensor_min_range_m=0.2,
            distance_clip_m=10.0,
            unknown_encoding_offset_m=20.0,
            minimum_history_frames=minimum_history_frames,
        ),
        partition,
        fov,
    )


class AngularPartitionTest(unittest.TestCase):
    def test_reference_shape_and_six_body_directions(self):
        partition = reference_partition()
        self.assertEqual(partition.spec.azimuth_bins, 80)
        self.assertEqual(partition.spec.elevation_bins, 40)
        self.assertEqual(partition.spec.number_of_bins, 3200)
        points = np.asarray(
            [
                [1.0, 0.0, 0.0],   # front +X
                [-1.0, 0.0, 0.0],  # rear -X
                [0.0, 1.0, 0.0],   # left +Y
                [0.0, -1.0, 0.0],  # right -Y
                [0.0, 0.0, 1.0],   # up +Z
                [0.0, 0.0, -1.0],  # down -Z
            ]
        )
        indices, _ = partition.indices(points)
        np.testing.assert_array_equal(indices, [1640, 1600, 1660, 1620, 3160, 40])

    def test_invalid_zero_nan_inf_and_range_boundaries(self):
        partition = reference_partition()
        points = np.asarray(
            [[0.0, 0.0, 0.0], [np.nan, 0.0, 0.0], [np.inf, 0.0, 0.0]]
        )
        indices, _ = partition.indices(points)
        np.testing.assert_array_equal(indices, [-1, -1, -1])

    def test_bin_order_is_elevation_major_azimuth_fast(self):
        partition = reference_partition()
        directions = partition.direction_centers()
        self.assertEqual(directions.shape, (3200, 3))
        first_row_elevations = np.degrees(np.arcsin(directions[:80, 2]))
        np.testing.assert_allclose(first_row_elevations, -87.75, atol=1.0e-10)


class PoseAndHistoryTest(unittest.TestCase):
    def test_pose_interpolation_uses_bracketing_poses(self):
        buffer = PoseBuffer(capacity=10, maximum_interpolation_gap_sec=0.2)
        buffer.add(pose(1.0, (0.0, 0.0, 0.0), 0.0))
        buffer.add(pose(1.1, (1.0, 0.0, 0.0), 90.0))
        interpolated, mode = buffer.lookup(1.05)
        self.assertEqual(mode, "interpolated")
        np.testing.assert_allclose(interpolated.translation, [0.5, 0.0, 0.0])
        forward_world = interpolated.body_to_world(np.asarray([[1.0, 0.0, 0.0]]))[0]
        np.testing.assert_allclose(
            forward_world, [0.5 + math.sqrt(0.5), math.sqrt(0.5), 0.0], atol=1.0e-6
        )

    def test_pose_lookup_does_not_substitute_latest_pose(self):
        buffer = PoseBuffer(capacity=10, maximum_interpolation_gap_sec=0.05)
        buffer.add(pose(1.0))
        found, reason = buffer.lookup(1.01)
        self.assertIsNone(found)
        self.assertEqual(reason, "cloud_newer_than_pose_history")

    def test_large_interpolation_gap_fails_closed(self):
        buffer = PoseBuffer(capacity=10, maximum_interpolation_gap_sec=0.05)
        buffer.add(pose(1.0))
        buffer.add(pose(1.2))
        found, reason = buffer.lookup(1.1)
        self.assertIsNone(found)
        self.assertEqual(reason, "pose_interpolation_gap_too_large")

    def test_ros_time_reset_clears_pose_and_cloud_history(self):
        pose_buffer = PoseBuffer(capacity=10, maximum_interpolation_gap_sec=0.05)
        pose_buffer.add(pose(10.0))
        self.assertTrue(pose_buffer.add(pose(1.0)))
        self.assertEqual(pose_buffer.size, 1)

        history = CloudHistoryBuffer(5)
        history.append(CloudFrame(10.0, "camera_init", np.zeros((1, 3)), pose(10.0)))
        self.assertTrue(
            history.append(CloudFrame(1.0, "camera_init", np.zeros((1, 3)), pose(1.0)))
        )
        self.assertEqual(history.size, 1)

    def test_causal_lookup_never_uses_future_pose(self):
        buffer = PoseBuffer(10, 0.05)
        buffer.add(pose(1.00, (0.0, 0.0, 0.0)))
        buffer.add(pose(1.02, (0.2, 0.0, 0.0)))
        selected, mode = buffer.lookup_at_or_before(1.01, 0.05)
        self.assertEqual(mode, "causal_previous")
        self.assertAlmostEqual(selected.stamp_sec, 1.00)
        np.testing.assert_allclose(selected.translation, [0.0, 0.0, 0.0])
        missing, reason = buffer.lookup_at_or_before(1.10, 0.05)
        self.assertIsNone(missing)
        self.assertEqual(reason, "causal_pose_too_old")

    def test_explicit_sensor_to_body_extrinsic(self):
        transformed = sensor_to_body(
            np.asarray([[1.0, 0.0, 0.0]], dtype=np.float64),
            [0.0, 0.0, 0.13],
            yaw_quaternion(90.0),
        )
        np.testing.assert_allclose(transformed, [[0.0, 1.0, 0.13]], atol=1e-8)


class HistoryAlignmentTest(unittest.TestCase):
    def test_translation_alignment_removes_naive_ghost(self):
        builder = reference_builder(minimum_history_frames=2)
        first = CloudFrame(1.0, "camera_init", np.asarray([[5.0, 0.0, 0.0]]), pose(1.0))
        second = CloudFrame(
            1.1, "camera_init", np.asarray([[4.0, 0.0, 0.0]]), pose(1.1, (1.0, 0.0, 0.0))
        )
        _, aligned, _ = builder.build([first, second], "body")
        # Correct alignment overlaps at 4 m; naive history concatenation would
        # retain both 5 m and 4 m surfaces.
        self.assertEqual(aligned.shape, (1, 3))
        np.testing.assert_allclose(aligned[0], [4.0, 0.0, 0.0], atol=0.05)

    def test_yaw_alignment_removes_rotational_ghost(self):
        builder = reference_builder(minimum_history_frames=2)
        first = CloudFrame(1.0, "camera_init", np.asarray([[5.0, 0.0, 0.0]]), pose(1.0))
        second = CloudFrame(
            1.1, "camera_init", np.asarray([[0.0, -5.0, 0.0]]), pose(1.1, yaw_deg=90.0)
        )
        _, aligned, _ = builder.build([first, second], "body")
        self.assertEqual(aligned.shape, (1, 3))
        np.testing.assert_allclose(aligned[0], [0.0, -5.0, 0.0], atol=0.05)


class SurrogateSemanticTest(unittest.TestCase):
    def test_obstacle_free_unknown_are_distinct_and_dimension_is_fixed(self):
        builder = reference_builder()
        frame = CloudFrame(
            1.0,
            "camera_init",
            np.asarray([[2.0, 0.0, 0.0]], dtype=np.float32),
            pose(1.0),
        )
        observation, _, _ = builder.build([frame], "body")
        partition = reference_partition()
        directions = np.asarray(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, -1.0]]
        )
        indices, _ = partition.indices(directions)
        front, left, down = indices
        self.assertEqual(observation.number_of_bins, 3200)
        self.assertEqual(observation.semantic[front], BinSemantic.KNOWN_OBSTACLE)
        self.assertAlmostEqual(observation.nearest_obstacle_distance[front], 2.0, places=5)
        self.assertEqual(observation.semantic[left], BinSemantic.OBSERVED_FREE)
        self.assertAlmostEqual(observation.lidar_surrogate[left], 10.0, places=5)
        self.assertEqual(observation.semantic[down], BinSemantic.UNKNOWN)
        self.assertGreater(observation.lidar_surrogate[down], 10.0)
        self.assertEqual(observation.lidar_valid_mask[down], 0.0)
        self.assertEqual(observation.unknown_mask[down], 1.0)

    def test_nearest_point_wins_and_out_of_range_is_rejected(self):
        builder = reference_builder()
        frame = CloudFrame(
            1.0,
            "camera_init",
            np.asarray([[4.0, 0.0, 0.0], [2.0, 0.0, 0.0], [11.0, 0.0, 0.0]]),
            pose(1.0),
        )
        observation, _, _ = builder.build([frame], "body")
        index = reference_partition().indices(np.asarray([[1.0, 0.0, 0.0]]))[0][0]
        self.assertAlmostEqual(observation.nearest_obstacle_distance[index], 2.0, places=5)

    def test_occlusion_marks_space_behind_return_unknown(self):
        builder = reference_builder()
        partition = reference_partition()
        direction = partition.direction_centers()[
            partition.indices(np.asarray([[1.0, 0.0, 0.0]]))[0][0]
        ]
        frame = CloudFrame(
            1.0,
            "camera_init",
            np.asarray([direction * 2.0]),
            pose(1.0),
        )
        observed_free_range = builder.unknown_estimator.estimate(
            direction.reshape(1, 3), [frame], frame.pose_world_body
        )
        self.assertLess(observed_free_range[0], 2.5)


class SourceTimestampPreservationTest(unittest.TestCase):
    def test_node_preserves_original_ros_stamp_without_float_round_trip(self):
        source_stamp = object()
        self.assertIs(preserve_source_stamp(source_stamp), source_stamp)


if __name__ == "__main__":
    unittest.main()
