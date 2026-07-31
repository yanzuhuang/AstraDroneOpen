#!/usr/bin/env python3

import math

from astra_swarm_manager.kinematics import (
    KinematicEstimator,
    bounded_prediction,
)


def test_position_differences_produce_bounded_velocity_and_acceleration():
    estimator = KinematicEstimator(
        velocity_alpha=1.0, acceleration_alpha=1.0,
        maximum_speed=3.0, maximum_acceleration=2.0)
    velocity, acceleration = estimator.update((0.0, 0.0, 0.0), 1.0)
    assert velocity == [0.0, 0.0, 0.0]
    assert acceleration == [0.0, 0.0, 0.0]

    velocity, acceleration = estimator.update((0.1, 0.0, 0.0), 1.1)
    assert abs(velocity[0] - 1.0) < 1e-9
    assert abs(acceleration[0] - 2.0) < 1e-9


def test_large_pose_jump_is_speed_limited():
    estimator = KinematicEstimator(
        velocity_alpha=1.0, maximum_speed=3.0)
    estimator.update((0.0, 0.0, 0.0), 1.0)
    velocity, _ = estimator.update((10.0, 0.0, 0.0), 1.1)
    assert abs(velocity[0] - 3.0) < 1e-9


def test_invalid_time_step_does_not_create_motion():
    estimator = KinematicEstimator(velocity_alpha=1.0)
    estimator.update((0.0, 0.0, 0.0), 1.0)
    velocity, acceleration = estimator.update((1.0, 0.0, 0.0), 1.005)
    assert velocity == [0.0, 0.0, 0.0]
    assert acceleration == [0.0, 0.0, 0.0]


def test_prediction_never_exceeds_planner_speed_limit():
    points = bounded_prediction(
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        4.0, 0.25, 0.2, 0.5)
    assert len(points) == 17
    for first, second in zip(points, points[1:]):
        speed = math.dist(first, second) / 0.25
        assert speed <= 0.2 + 1e-9
    assert points[-1][0] <= 0.8


def test_prediction_preserves_stationary_hover():
    points = bounded_prediction(
        (1.0, 2.0, 3.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        1.0, 0.25, 0.2, 0.5)
    assert points == [(1.0, 2.0, 3.0)] * 5
