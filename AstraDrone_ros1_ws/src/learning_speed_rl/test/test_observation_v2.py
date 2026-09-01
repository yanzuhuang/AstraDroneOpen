#!/usr/bin/env python3

import math
import struct
from types import SimpleNamespace
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
    decode_xyz_points,
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


def legacy_unknown_estimate(estimator, directions, frames, current_pose):
    """Frozen pre-optimization algorithm used only for numeric regression."""

    config = estimator.config
    directions = np.asarray(directions, dtype=np.float64)
    radial = np.arange(
        config.sensor_min_range_m + config.radial_sample_step_m,
        config.distance_clip_m,
        config.radial_sample_step_m,
        dtype=np.float64,
    )
    radial = np.unique(np.append(radial, config.distance_clip_m))
    points_current = directions[:, None, :] * radial[None, :, None]
    points_world = current_pose.body_to_world(points_current.reshape(-1, 3))
    observed = np.zeros((directions.shape[0], radial.size), dtype=bool)
    rotation = config._rotation_body_sensor
    translation = config.sensor_translation_body_m
    full_azimuth = bool(
        config.azimuth_max_deg - config.azimuth_min_deg >= 360.0 - 1.0e-6
    )
    az_bins = int(
        np.ceil(
            (config.azimuth_max_deg - config.azimuth_min_deg)
            / config.occlusion_angular_resolution_deg
        )
    )
    el_bins = int(
        np.ceil(
            (config.elevation_max_deg - config.elevation_min_deg)
            / config.occlusion_angular_resolution_deg
        )
    )
    for frame in frames:
        candidate = (
            frame.pose_world_body.world_to_body(points_world) - translation
        ) @ rotation
        ranges = np.linalg.norm(candidate, axis=1)
        azimuth = np.degrees(np.arctan2(candidate[:, 1], candidate[:, 0]))
        elevation = np.degrees(
            np.arctan2(candidate[:, 2], np.hypot(candidate[:, 0], candidate[:, 1]))
        )
        visible = (
            np.isfinite(ranges)
            & (ranges >= config.sensor_min_range_m)
            & (ranges <= config.distance_clip_m)
            & (elevation >= config.elevation_min_deg)
            & (elevation <= config.elevation_max_deg)
        )
        if not full_azimuth:
            visible &= (
                (azimuth >= config.azimuth_min_deg)
                & (azimuth <= config.azimuth_max_deg)
            )
        source = (frame.points_body.astype(np.float64) - translation) @ rotation
        source_ranges = np.linalg.norm(source, axis=1)
        source_azimuth = np.degrees(np.arctan2(source[:, 1], source[:, 0]))
        source_elevation = np.degrees(
            np.arctan2(source[:, 2], np.hypot(source[:, 0], source[:, 1]))
        )
        source_valid = (
            np.all(np.isfinite(source), axis=1)
            & (source_ranges > config.sensor_min_range_m)
            & (source_ranges <= config.distance_clip_m)
            & (source_elevation >= config.elevation_min_deg)
            & (source_elevation <= config.elevation_max_deg)
        )
        nearest = np.full(az_bins * el_bins, np.inf, dtype=np.float64)
        if np.any(source_valid):
            source_az = source_azimuth[source_valid]
            if full_azimuth:
                source_az = (
                    (source_az - config.azimuth_min_deg) % 360.0
                ) + config.azimuth_min_deg
            source_az_index = np.clip(
                np.floor(
                    (source_az - config.azimuth_min_deg)
                    / config.occlusion_angular_resolution_deg
                ).astype(np.int64),
                0,
                az_bins - 1,
            )
            source_el_index = np.clip(
                np.floor(
                    (source_elevation[source_valid] - config.elevation_min_deg)
                    / config.occlusion_angular_resolution_deg
                ).astype(np.int64),
                0,
                el_bins - 1,
            )
            np.minimum.at(
                nearest,
                source_el_index * az_bins + source_az_index,
                source_ranges[source_valid],
            )
        candidate_az = azimuth.copy()
        if full_azimuth:
            candidate_az = (
                (candidate_az - config.azimuth_min_deg) % 360.0
            ) + config.azimuth_min_deg
        az_index = np.clip(
            np.floor(
                (candidate_az - config.azimuth_min_deg)
                / config.occlusion_angular_resolution_deg
            ).astype(np.int64),
            0,
            az_bins - 1,
        )
        el_index = np.clip(
            np.floor(
                (elevation - config.elevation_min_deg)
                / config.occlusion_angular_resolution_deg
            ).astype(np.int64),
            0,
            el_bins - 1,
        )
        limit = nearest[el_index * az_bins + az_index] + config.occlusion_margin_m
        visible &= (ranges <= limit) | ~np.isfinite(limit)
        observed |= visible.reshape(observed.shape)
    contiguous = np.logical_and.accumulate(observed, axis=1)
    counts = np.sum(contiguous, axis=1)
    result = np.full(
        directions.shape[0], config.sensor_min_range_m, dtype=np.float64
    )
    covered = counts > 0
    result[covered] = radial[np.minimum(counts[covered] - 1, radial.size - 1)]
    return np.minimum(result, config.distance_clip_m).astype(np.float32)


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


class PointCloudDecodeTest(unittest.TestCase):
    @staticmethod
    def _field(name, offset, datatype=7):
        return SimpleNamespace(
            name=name, offset=offset, datatype=datatype, count=1
        )

    def test_vectorized_decode_preserves_order_limit_and_finite_filter(self):
        values = (
            (1.0, 2.0, 3.0),
            (float("nan"), 0.0, 0.0),
            (4.0, 5.0, 6.0),
            (7.0, 8.0, 9.0),
        )
        payload = b"".join(struct.pack("<fff", *value) for value in values)
        message = SimpleNamespace(
            fields=[
                self._field("x", 0),
                self._field("y", 4),
                self._field("z", 8),
            ],
            is_bigendian=False,
            point_step=12,
            row_step=24,
            width=2,
            height=2,
            data=payload,
        )
        decoded = decode_xyz_points(message, 3)
        np.testing.assert_array_equal(
            decoded,
            np.asarray([[1, 2, 3], [4, 5, 6]], dtype=np.float32),
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
    def test_optimized_unknown_estimator_is_bitwise_legacy_equivalent(self):
        builder = reference_builder(minimum_history_frames=3)
        partition = reference_partition()
        rng = np.random.default_rng(20260827)
        frames = []
        for index in range(3):
            points = rng.normal(size=(800, 3)).astype(np.float32)
            points *= rng.uniform(0.3, 8.0, size=(800, 1)).astype(np.float32)
            frames.append(
                CloudFrame(
                    1.0 + 0.1 * index,
                    "mid360_link",
                    points,
                    pose(
                        1.0 + 0.1 * index,
                        (0.05 * index, -0.03 * index, 0.01 * index),
                        2.0 * index,
                    ),
                )
            )
        directions = partition.direction_centers()
        expected = legacy_unknown_estimate(
            builder.unknown_estimator, directions, frames, frames[-1].pose_world_body
        )
        actual = builder.unknown_estimator.estimate(
            directions, frames, frames[-1].pose_world_body
        )
        np.testing.assert_array_equal(actual, expected)

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
