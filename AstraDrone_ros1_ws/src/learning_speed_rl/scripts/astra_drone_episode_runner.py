#!/usr/bin/env python3
"""Episode v0.1 owner for the reviewed UAV1 mock-action ingress."""

import json
import math
import os
from pathlib import Path
import time

import rospy

from learning_speed_rl.training import (
    AstraDroneEnv,
    AstraDroneEpisodeConfig,
)


def _episode_output_path(value):
    path = Path(str(value)).expanduser().resolve()
    repository_root = next(
        (
            parent
            for parent in Path(__file__).resolve().parents
            if (parent / "runtime_artifacts").is_dir()
            and (parent / "AstraDrone_ros1_ws").is_dir()
        ),
        None,
    )
    if repository_root is None:
        raise rospy.ROSInitException("AstraDroneOpen repository root was not found")
    artifact_root = (repository_root / "runtime_artifacts").resolve()
    if artifact_root not in path.parents:
        raise rospy.ROSInitException(
            "Episode output must be below {}".format(artifact_root)
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _write_json(path, value):
    temporary = str(path) + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    os.replace(temporary, str(path))


def _finite_action_sequence(values):
    if not isinstance(values, list) or not values:
        raise rospy.ROSInitException(
            "validation/action_sequence_mps must be a non-empty list"
        )
    result = [float(value) for value in values]
    if not all(math.isfinite(value) and value > 0.0 for value in result):
        raise rospy.ROSInitException("validation action sequence is invalid")
    return result


def main():
    rospy.init_node("astra_drone_episode_v01", anonymous=False)
    output_path = _episode_output_path(rospy.get_param("~output_file"))
    actions = _finite_action_sequence(
        rospy.get_param("~validation/action_sequence_mps")
    )
    max_steps_value = rospy.get_param("~episode/max_steps", len(actions))
    max_duration_value = rospy.get_param("~episode/max_duration_sec", None)
    config = AstraDroneEpisodeConfig(
        policy_period_sec=float(
            rospy.get_param("~episode/policy_period_sec", 0.1)
        ),
        max_steps=(None if max_steps_value is None else int(max_steps_value)),
        max_duration_sec=(
            None
            if max_duration_value is None
            else float(max_duration_value)
        ),
        start_timeout_sec=float(
            rospy.get_param("~episode/start_timeout_sec", 600.0)
        ),
        completion_timeout_sec=float(
            rospy.get_param("~episode/completion_timeout_sec", 30.0)
        ),
        deadline_tolerance_sec=float(
            rospy.get_param("~episode/deadline_tolerance_sec", 0.02)
        ),
        maximum_in_flight=int(
            rospy.get_param("~episode/maximum_in_flight", 16)
        ),
        minimum_active_speed_mps=float(
            rospy.get_param("~episode/minimum_active_speed_mps", 0.2)
        ),
    )

    deadline = time.monotonic() + 60.0
    while (
        rospy.Time.now().to_sec() <= 0.0
        and time.monotonic() < deadline
        and not rospy.is_shutdown()
    ):
        time.sleep(0.05)
    if rospy.Time.now().to_sec() <= 0.0:
        raise rospy.ROSInitException(
            "positive ROS/simulation time was not available"
        )

    if not bool(
        rospy.get_param(
            "/uav1/speed_adapter/timing/mock_request_driven", False
        )
    ):
        raise rospy.ROSInitException(
            "Episode v0.1 requires UAV1 SpeedAdapter mock_request_driven=true"
        )

    env = AstraDroneEnv.from_ros_params()
    result = {
        "schema_version": "astra_drone_episode_runtime_v0.1",
        "started_wall_iso": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "configured_action_sequence_mps": actions,
        "configured_episode": {
            "policy_period_sec": config.policy_period_sec,
            "max_steps": config.max_steps,
            "max_duration_sec": config.max_duration_sec,
            "deadline_tolerance_sec": config.deadline_tolerance_sec,
            "maximum_in_flight": config.maximum_in_flight,
            "minimum_active_speed_mps": config.minimum_active_speed_mps,
        },
    }
    _write_json(output_path, result)
    try:
        episode = env.run_episode(
            lambda _state, index: actions[index % len(actions)], config
        )
        result.update(episode)
    except Exception as error:
        result.update(
            {
                "status": "runner_error",
                "error": "{}: {}".format(type(error).__name__, error),
            }
        )
        rospy.logerr("Episode v0.1 runner failed: %s", result["error"])
    result["finished_wall_iso"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
    result["final_readiness"] = env.episode_readiness()
    _write_json(output_path, result)

    if result.get("status") == "completed":
        rospy.logwarn(
            "Episode v0.1 complete: actions=%d transitions=%d terminated=%s truncated=%s",
            result["metrics"]["scheduled_action_count"],
            result["metrics"]["transition_count"],
            result["episode"]["terminated"],
            result["episode"]["truncated"],
        )
        return 0
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
