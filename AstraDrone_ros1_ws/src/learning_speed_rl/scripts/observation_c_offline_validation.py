#!/usr/bin/env python3
"""Write deterministic Scheme C geometry/speed-sampling evidence as JSON."""

import argparse
import json
import math
import os

import numpy as np

from learning_speed_rl.observation.scheme_c import (
    EgoBsplineTrajectory,
    KinematicState,
    LidarSurrogateFeature,
    ObservationCBuilder,
    TrajectorySampler,
    TrajectorySamplingConfig,
)
from learning_speed_rl.observation.v2 import Pose3D
from learning_speed_rl.policy import MockSpeedPolicy


def trajectory(points, scale=1.0, start=10.0, trajectory_id=1):
    return EgoBsplineTrajectory(
        3, np.asarray(points, dtype=np.float64),
        np.arange(-3.0, len(points) + 1.0) * scale,
        start, trajectory_id, "camera_init",
    )


def feature(stamp=10.0):
    return LidarSurrogateFeature(
        stamp, "body", np.ones(3200), np.ones(3200),
        np.zeros(3200), np.ones(3200, dtype=np.uint8),
    )


def kinematic(stamp, position, yaw_deg=0.0):
    half = math.radians(yaw_deg) * 0.5
    pose = Pose3D(stamp, position, [0.0, 0.0, math.sin(half), math.cos(half)])
    return KinematicState(pose, [1.0, 0.0, 0.0], True)


def build(mode, traj, pose_state, previous_v_max=0.20):
    builder = ObservationCBuilder(
        TrajectorySampler(TrajectorySamplingConfig(20, 0.25, 5.0, mode)),
        "camera_init", "body",
    )
    return builder.build(feature(), traj, pose_state, previous_v_max)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default="runtime_artifacts/scheme_c_trajectory_fusion/offline_validation.json",
    )
    args = parser.parse_args()
    line_points = [[float(index), 0.0, 0.0] for index in range(8)]
    turn_points = [[0, 0, 0], [1, 0, 0], [2, 0, 0], [3, 0, 0],
                   [3, 1, 0], [3, 2, 0], [3, 3, 0], [3, 4, 0]]
    fast = trajectory(line_points, 1.0)
    slow = trajectory(line_points, 2.0, trajectory_id=2)
    mock_policy = MockSpeedPolicy(0.20)
    fast_v_max = mock_policy.predict(None)
    mock_policy.set_command(0.10)
    slow_v_max = mock_policy.predict(None)
    current = fast.evaluate_elapsed(0.0)
    straight = build("distance", fast, kinematic(10.0, current), fast_v_max)
    yawed = build("distance", fast, kinematic(10.0, current, 90.0))
    turning_traj = trajectory(turn_points)
    turning = build(
        "distance", turning_traj,
        kinematic(10.0, turning_traj.evaluate_elapsed(0.0)),
    )
    slow_current = slow.evaluate_elapsed(0.0)
    fast_distance = straight.future_trajectory.positions_body
    slow_distance = build(
        "distance", slow, kinematic(10.0, slow_current), slow_v_max
    ).future_trajectory.positions_body
    fast_time = build("time", fast, kinematic(10.0, current)).future_trajectory.positions_body
    slow_time = build(
        "time", slow, kinematic(10.0, slow_current), slow_v_max
    ).future_trajectory.positions_body
    result = {
        "contract": "scheme_c_trajectory_fusion_v1.0",
        "mock_fast_v_max_mps": fast_v_max,
        "mock_slow_v_max_mps": slow_v_max,
        "straight_min_forward_x_m": float(np.min(straight.future_trajectory.positions_body[:, 0])),
        "turn_max_left_y_m": float(np.max(turning.future_trajectory.positions_body[:, 1])),
        "yaw_90_max_abs_body_x_m": float(np.max(np.abs(yawed.future_trajectory.positions_body[:, 0]))),
        "yaw_90_max_body_right_m": float(-np.min(yawed.future_trajectory.positions_body[:, 1])),
        "distance_sampling_time_scale_l2_delta_m": float(np.linalg.norm(fast_distance - slow_distance)),
        "time_sampling_time_scale_l2_delta_m": float(np.linalg.norm(fast_time - slow_time)),
        "observation_valid": bool(straight.ready_for_policy),
        "ego_core_modified": False,
    }
    output = os.path.abspath(args.output)
    os.makedirs(os.path.dirname(output), exist_ok=True)
    with open(output, "w", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(output)


if __name__ == "__main__":
    main()
