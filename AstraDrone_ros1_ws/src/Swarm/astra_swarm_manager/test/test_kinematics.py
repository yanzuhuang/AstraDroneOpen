#!/usr/bin/env python3

from astra_swarm_manager.kinematics import KinematicEstimator


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
