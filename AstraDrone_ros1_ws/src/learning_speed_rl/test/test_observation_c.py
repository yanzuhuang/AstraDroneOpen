#!/usr/bin/env python3

import math
import os
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from learning_speed_rl.observation.scheme_c import (
    ActiveTrajectoryStore,
    EgoBsplineTrajectory,
    KinematicState,
    KinematicStateBuffer,
    LidarSurrogateFeature,
    ObservationCBuilder,
    TrajectorySampler,
    TrajectorySamplingConfig,
    TimestampedScalarBuffer,
)
from learning_speed_rl.observation.v2 import Pose3D
from learning_speed_rl.policy import MockSpeedPolicy


def yaw_quaternion(degrees):
    half = math.radians(degrees) * 0.5
    return np.asarray([0.0, 0.0, math.sin(half), math.cos(half)])


def make_trajectory(points=None, time_scale=1.0, start=10.0, trajectory_id=1):
    if points is None:
        points = np.asarray([[float(i), 0.0, 0.0] for i in range(8)])
    knots = np.arange(-3.0, len(points) + 1.0) * time_scale
    return EgoBsplineTrajectory(
        degree=3,
        control_points=np.asarray(points, dtype=np.float64),
        knots=knots,
        start_time_sec=start,
        trajectory_id=trajectory_id,
        frame_id="camera_init",
    )


def lidar(stamp):
    return LidarSurrogateFeature(
        stamp_sec=stamp,
        frame_id="body",
        surrogate=np.ones(16),
        valid_mask=np.ones(16),
        unknown_mask=np.zeros(16),
        semantic=np.ones(16, dtype=np.uint8),
    )


def state(stamp, position, yaw_deg=0.0, velocity=(1.0, 0.0, 0.0)):
    return KinematicState(
        Pose3D(stamp, np.asarray(position), yaw_quaternion(yaw_deg)),
        np.asarray(velocity),
        True,
    )


def builder(mode="distance", count=8, spacing=0.5, max_distance=4.0):
    return ObservationCBuilder(
        TrajectorySampler(
            TrajectorySamplingConfig(
                sample_count=count,
                sample_spacing=spacing,
                max_distance=max_distance,
                sampling_mode=mode,
            )
        ),
        "camera_init",
        "body",
    )


class EgoBsplineTest(unittest.TestCase):
    def test_matches_uniform_cubic_line_and_formal_derivatives(self):
        trajectory = make_trajectory()
        np.testing.assert_allclose(trajectory.evaluate_elapsed(0.0), [1.0, 0.0, 0.0])
        np.testing.assert_allclose(trajectory.evaluate_elapsed(2.5), [3.5, 0.0, 0.0])
        np.testing.assert_allclose(trajectory.velocity_elapsed(1.0), [1.0, 0.0, 0.0])
        np.testing.assert_allclose(trajectory.acceleration_elapsed(1.0), [0.0, 0.0, 0.0])

    def test_rejects_malformed_official_message_content(self):
        with self.assertRaises(ValueError):
            EgoBsplineTrajectory(3, np.zeros((4, 3)), np.arange(7), 10.0, 1, "camera_init")
        with self.assertRaises(ValueError):
            make_trajectory(start=0.0)


class TrajectorySamplingTest(unittest.TestCase):
    def test_distance_horizon_matches_full_polyline_and_skips_unused_tail(self):
        points = np.asarray(
            [[float(index), 0.0, 0.0] for index in range(64)],
            dtype=np.float64,
        )
        trajectory = make_trajectory(points)
        sampler = TrajectorySampler(
            TrajectorySamplingConfig(
                sample_count=20,
                sample_spacing=0.25,
                max_distance=5.0,
                sampling_mode="distance",
                arc_length_resolution_m=0.02,
            )
        )
        elapsed = trajectory.elapsed_at(trajectory.start_time_sec)
        full_times, full_points, full_clipped = sampler._adaptive_polyline(
            trajectory, elapsed
        )
        full_lengths = np.linalg.norm(np.diff(full_points, axis=0), axis=1)
        full_cumulative = np.concatenate(([0.0], np.cumsum(full_lengths)))
        query = 0.25 * np.arange(1, 21, dtype=np.float64)
        expected = np.empty((20, 3), dtype=np.float64)
        for index, distance in enumerate(query):
            upper = int(np.searchsorted(full_cumulative, distance, side="left"))
            lower = upper - 1
            span = full_cumulative[upper] - full_cumulative[lower]
            ratio = (distance - full_cumulative[lower]) / span
            expected[index] = (
                (1.0 - ratio) * full_points[lower]
                + ratio * full_points[upper]
            )

        sampled, offsets, metadata = sampler.sample(
            trajectory, trajectory.start_time_sec
        )

        self.assertFalse(full_clipped)
        self.assertTrue(metadata["arc_length_horizon_clipped"])
        self.assertTrue(metadata["remaining_arc_length_is_lower_bound"])
        np.testing.assert_array_equal(offsets, query)
        np.testing.assert_array_equal(sampled, expected)
        self.assertLess(
            metadata["arc_length_polyline_points"],
            len(full_times) // 4,
        )

    def test_distance_sampling_straight_points_are_forward(self):
        trajectory = make_trajectory()
        actual = trajectory.evaluate_elapsed(0.0)
        observation = builder().build(lidar(10.0), trajectory, state(10.0, actual), 0.2)
        points = observation.future_trajectory.positions_body
        self.assertTrue(np.all(points[:, 0] > 0.0))
        np.testing.assert_allclose(points[:, 1:], 0.0, atol=1.0e-6)

    def test_turn_direction_is_preserved(self):
        points = np.asarray(
            [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0],
             [3, 1, 0], [3, 2, 0], [3, 3, 0], [3, 4, 0]],
            dtype=np.float64,
        )
        trajectory = make_trajectory(points)
        actual = trajectory.evaluate_elapsed(0.0)
        feature = builder(count=10).build(
            lidar(10.0), trajectory, state(10.0, actual), 0.2
        ).future_trajectory.positions_body
        self.assertGreater(float(np.max(feature[:, 1])), 0.5)
        self.assertGreater(float(feature[0, 0]), 0.0)

    def test_world_trajectory_rotates_with_current_yaw(self):
        trajectory = make_trajectory()
        actual = trajectory.evaluate_elapsed(0.0)
        points_yaw_zero = builder().build(
            lidar(10.0), trajectory, state(10.0, actual, 0.0), 0.2
        ).future_trajectory.positions_body
        points_yaw_ninety = builder().build(
            lidar(10.0), trajectory, state(10.0, actual, 90.0), 0.2
        ).future_trajectory.positions_body
        np.testing.assert_allclose(points_yaw_ninety[:, 0], 0.0, atol=1.0e-5)
        np.testing.assert_allclose(points_yaw_ninety[:, 1], -points_yaw_zero[:, 0], atol=1.0e-5)

    def test_distance_sampling_is_more_time_scale_stable_than_time_sampling(self):
        mock_policy = MockSpeedPolicy(0.20)
        fast_v_max = mock_policy.predict(None)
        mock_policy.set_command(0.10)
        slow_v_max = mock_policy.predict(None)
        fast = make_trajectory(time_scale=1.0)
        slow = make_trajectory(time_scale=2.0)
        distance = builder("distance", count=5, spacing=0.5, max_distance=2.5)
        time = builder("time", count=5, spacing=0.5, max_distance=2.5)
        fast_state = state(10.0, fast.evaluate_elapsed(0.0))
        slow_state = state(10.0, slow.evaluate_elapsed(0.0))
        fast_distance = distance.build(lidar(10.0), fast, fast_state, fast_v_max)
        slow_distance = distance.build(lidar(10.0), slow, slow_state, slow_v_max)
        fast_time = time.build(lidar(10.0), fast, fast_state, fast_v_max)
        slow_time = time.build(lidar(10.0), slow, slow_state, slow_v_max)
        distance_delta = np.linalg.norm(
            fast_distance.future_trajectory.positions_body
            - slow_distance.future_trajectory.positions_body
        )
        time_delta = np.linalg.norm(
            fast_time.future_trajectory.positions_body
            - slow_time.future_trajectory.positions_body
        )
        self.assertLess(distance_delta, 1.0e-5)
        self.assertGreater(time_delta, 0.5)


class TimestampReplanningAndStateTest(unittest.TestCase):
    def test_previous_v_max_never_uses_a_future_update(self):
        buffer = TimestampedScalarBuffer(10)
        buffer.add(10.0, 0.20)
        buffer.add(11.0, 0.08)
        value, mode = buffer.lookup(10.5)
        self.assertAlmostEqual(value, 0.20)
        self.assertTrue(mode.startswith("zero_order_hold"))
        value, _ = buffer.lookup(11.0)
        self.assertAlmostEqual(value, 0.08)
        missing, reason = buffer.lookup(9.9)
        self.assertIsNone(missing)
        self.assertEqual(reason, "scalar_state_newer_than_observation")

    def test_timestamp_mismatch_fails_closed(self):
        trajectory = make_trajectory()
        with self.assertRaisesRegex(ValueError, "timestamp"):
            builder().build(lidar(9.9), trajectory, state(9.9, [1, 0, 0]), 0.2)
        with self.assertRaisesRegex(ValueError, "timestamp"):
            builder().build(lidar(16.0), trajectory, state(16.0, [6, 0, 0]), 0.2)

    def test_replanning_atomically_replaces_and_rejects_old_cache(self):
        store = ActiveTrajectoryStore()
        old = make_trajectory(start=10.0, trajectory_id=4)
        shifted = np.asarray([[i, 2.0, 0.0] for i in range(8)], dtype=np.float64)
        new = make_trajectory(shifted, start=11.0, trajectory_id=5)
        self.assertTrue(store.update(old))
        self.assertTrue(store.update(new))
        self.assertIs(store.current, new)
        self.assertIs(store.lookup(10.5), old)
        self.assertIs(store.lookup(11.5), new)
        self.assertFalse(store.update(old))
        self.assertIs(store.current, new)
        current = new.evaluate_elapsed(0.0)
        observation = builder().build(lidar(11.0), store.current, state(11.0, current), 0.2)
        self.assertEqual(observation.future_trajectory.trajectory_id, 5)

    def test_trajectory_history_is_bounded_and_fails_closed_outside_coverage(self):
        store = ActiveTrajectoryStore(capacity=1)
        old = make_trajectory(start=10.0, trajectory_id=4)
        new = make_trajectory(start=11.0, trajectory_id=5)
        self.assertTrue(store.update(old))
        self.assertTrue(store.update(new))
        self.assertIsNone(store.lookup(10.5))
        self.assertIs(store.lookup(11.5), new)
        self.assertIsNone(store.lookup(20.0))

    def test_future_trajectory_is_never_selected(self):
        store = ActiveTrajectoryStore()
        future = make_trajectory(start=11.0, trajectory_id=5)
        self.assertTrue(store.update(future))
        selected, diagnostic = store.lookup_with_diagnostics(10.9)
        self.assertIsNone(selected)
        self.assertEqual(diagnostic.result, "before_first_trajectory")

    def test_delayed_lookup_retains_exact_state_history(self):
        buffer = KinematicStateBuffer(10, 0.05, 1.0, 0.5)
        for stamp in (1.0, 1.1, 1.2):
            buffer.add_pose(
                Pose3D(stamp, [stamp, 0, 0], yaw_quaternion(0)),
                receipt_sec=stamp + 0.01,
            )
        delayed, result, diagnostic = buffer.lookup_with_diagnostics(1.1, 1.25)
        self.assertIsNotNone(delayed)
        self.assertEqual(result, "exact")
        self.assertEqual(diagnostic.before_stamp_sec, 1.1)
        self.assertEqual(diagnostic.after_stamp_sec, 1.1)
        self.assertAlmostEqual(diagnostic.nearest_state_dt_sec, 0.0)

    def test_interpolation_boundary_and_gap_failure_diagnostics(self):
        buffer = KinematicStateBuffer(10, 0.05, 1.0, 0.5)
        buffer.add_pose(Pose3D(0.95, [-0.05, 0, 0], yaw_quaternion(0)), 0.96)
        buffer.add_pose(Pose3D(1.0, [0, 0, 0], yaw_quaternion(0)), 1.01)
        buffer.add_pose(Pose3D(1.05, [0.05, 0, 0], yaw_quaternion(0)), 1.06)
        state_at_boundary, result, diagnostic = buffer.lookup_with_diagnostics(
            1.025, 1.07
        )
        self.assertIsNotNone(state_at_boundary)
        self.assertEqual(result, "interpolated")
        self.assertAlmostEqual(diagnostic.bracket_span_sec, 0.05)

        too_wide = KinematicStateBuffer(10, 0.05, 1.0, 0.5)
        too_wide.add_pose(Pose3D(0.9, [-0.1, 0, 0], yaw_quaternion(0)))
        too_wide.add_pose(Pose3D(1.0, [0, 0, 0], yaw_quaternion(0)))
        too_wide.add_pose(
            Pose3D(1.050002, [0.05, 0, 0], yaw_quaternion(0))
        )
        missing, result, diagnostic = too_wide.lookup_with_diagnostics(1.025)
        self.assertIsNone(missing)
        self.assertEqual(result, "kinematic_interpolation_gap_too_large")
        self.assertGreater(diagnostic.bracket_span_sec, 0.05)
        self.assertFalse(diagnostic.before_stamp_sec is None)
        self.assertFalse(diagnostic.after_stamp_sec is None)

    def test_before_after_missing_eviction_and_out_of_order_evidence(self):
        buffer = KinematicStateBuffer(2, 0.05, 1.0, 0.5)
        buffer.add_pose(Pose3D(1.0, [0, 0, 0], yaw_quaternion(0)))
        buffer.add_pose(Pose3D(1.1, [0.1, 0, 0], yaw_quaternion(0)))
        before_missing, result, diagnostic = buffer.lookup_with_diagnostics(0.9)
        self.assertIsNone(before_missing)
        self.assertEqual(result, "observation_precedes_kinematic_history")
        self.assertIsNone(diagnostic.before_stamp_sec)
        self.assertEqual(diagnostic.after_stamp_sec, 1.0)

        after_missing, result, diagnostic = buffer.lookup_with_diagnostics(1.2)
        self.assertIsNone(after_missing)
        self.assertEqual(result, "observation_newer_than_kinematic_history")
        self.assertEqual(diagnostic.before_stamp_sec, 1.1)
        self.assertIsNone(diagnostic.after_stamp_sec)

        buffer.add_pose(Pose3D(1.2, [0.2, 0, 0], yaw_quaternion(0)))
        evicted, _, diagnostic = buffer.lookup_with_diagnostics(1.0)
        self.assertIsNone(evicted)
        self.assertEqual(diagnostic.eviction_count, 1)
        self.assertEqual(diagnostic.oldest_stamp_sec, 1.1)

        self.assertTrue(
            buffer.add_pose(Pose3D(0.5, [0, 0, 0], yaw_quaternion(0)))
        )
        _, _, diagnostic = buffer.lookup_with_diagnostics(0.5)
        self.assertEqual(diagnostic.out_of_order_state_count, 1)
        self.assertEqual(diagnostic.buffer_size, 1)

    def test_backend_neutral_position_difference_and_interpolation(self):
        buffer = KinematicStateBuffer(10, 0.15, 1.0, 0.5)
        buffer.add_pose(Pose3D(1.0, np.asarray([0.0, 0.0, 0.0]), yaw_quaternion(0)))
        buffer.add_pose(Pose3D(1.1, np.asarray([0.1, 0.0, 0.0]), yaw_quaternion(0)))
        buffer.add_pose(Pose3D(1.2, np.asarray([0.2, 0.0, 0.0]), yaw_quaternion(0)))
        exact, mode = buffer.lookup(1.2)
        self.assertEqual(mode, "exact")
        np.testing.assert_allclose(exact.velocity_world, [1.0, 0.0, 0.0])
        buffer.add_pose(Pose3D(1.2, np.asarray([0.2, 0.0, 0.0]), yaw_quaternion(0)))
        duplicate, mode = buffer.lookup(1.2)
        self.assertEqual(mode, "exact")
        np.testing.assert_allclose(duplicate.velocity_world, [1.0, 0.0, 0.0])
        interpolated, mode = buffer.lookup(1.15)
        self.assertEqual(mode, "interpolated")
        np.testing.assert_allclose(interpolated.pose.translation, [0.15, 0.0, 0.0])
        missing, reason = buffer.lookup(1.21)
        self.assertIsNone(missing)
        self.assertEqual(reason, "observation_newer_than_kinematic_history")

    def test_training_causal_state_lookup_never_uses_future_sample(self):
        buffer = KinematicStateBuffer(
            10, 0.05, 1.0, 0.5, lookup_policy="causal_at_or_before"
        )
        buffer.add_pose(Pose3D(1.00, [0.0, 0.0, 0.0], yaw_quaternion(0)))
        buffer.add_pose(Pose3D(1.01, [0.1, 0.0, 0.0], yaw_quaternion(0)))
        buffer.add_pose(Pose3D(1.02, [0.2, 0.0, 0.0], yaw_quaternion(0)))
        selected, mode, diagnostic = buffer.lookup_with_diagnostics(1.015)
        self.assertIsNotNone(selected)
        self.assertEqual(mode, "causal_previous")
        self.assertAlmostEqual(selected.pose.stamp_sec, 1.01)
        self.assertIsNone(diagnostic.after_stamp_sec)
        self.assertGreaterEqual(diagnostic.dt_before_sec, 0.0)

    def test_training_truth_provenance_does_not_change_values(self):
        trajectory = make_trajectory()
        actual = trajectory.evaluate_elapsed(0.0)
        kinematic = state(10.0, actual, velocity=(0.4, 0.1, 0.0))
        default_observation = builder().build(
            lidar(10.0), trajectory, kinematic, 0.4
        )
        truth_builder = ObservationCBuilder(
            builder().sampler,
            "camera_init",
            "body",
            state_source_type="gazebo_truth_training",
            state_contract_version="astradrone_training_odometry_v1.0",
            state_velocity_source="timestamped_state_position_difference_world",
        )
        truth_observation = truth_builder.build(
            lidar(10.0), trajectory, kinematic, 0.4
        )
        np.testing.assert_allclose(
            truth_observation.system_state.actual_velocity_body,
            default_observation.system_state.actual_velocity_body,
        )
        np.testing.assert_allclose(
            truth_observation.system_state.tracking_error_body,
            default_observation.system_state.tracking_error_body,
        )
        self.assertEqual(
            truth_observation.metadata["state_source_type"],
            "gazebo_truth_training",
        )
        self.assertNotIn(
            "fast_lio", truth_observation.metadata["tracking_error_definition"]
        )

    def test_invalid_lidar_and_system_state_do_not_build(self):
        with self.assertRaises(ValueError):
            LidarSurrogateFeature(10.0, "body", [np.nan], [1], [0], [1])
        trajectory = make_trajectory()
        with self.assertRaises(ValueError):
            builder().build(
                lidar(10.0), trajectory,
                KinematicState(Pose3D(10.0, [1, 0, 0], yaw_quaternion(0)), [np.nan, 0, 0], True),
                0.2,
            )
        with self.assertRaises(ValueError):
            builder().build(lidar(10.0), trajectory, state(10.0, [1, 0, 0]), float("nan"))


class NamespaceIsolationTest(unittest.TestCase):
    def test_configured_topics_are_relative_and_launch_uses_namespace_group(self):
        package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        config_path = os.path.join(package_dir, "config", "observation_c_trajectory_fusion.yaml")
        launch_path = os.path.join(package_dir, "launch", "observation_c_trajectory_fusion.launch")
        with open(config_path, "r", encoding="utf-8") as stream:
            topics = yaml.safe_load(stream)["observation_c"]["topics"]
        self.assertTrue(all(value and not value.startswith("/") for value in topics.values()))
        tree = ET.parse(launch_path)
        groups = tree.getroot().findall("group")
        self.assertTrue(any(group.attrib.get("ns") == "$(arg namespace)" for group in groups))


if __name__ == "__main__":
    unittest.main()
