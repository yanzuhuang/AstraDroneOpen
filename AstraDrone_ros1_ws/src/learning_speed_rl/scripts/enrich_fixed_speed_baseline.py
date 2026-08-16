#!/usr/bin/env python3
"""Normalize planner collision semantics in completed baseline artifacts."""

import argparse
import glob
import json
import os
from collections import defaultdict

import rosbag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    pattern = os.path.join(os.path.abspath(args.root), "runs", "*", "baseline.bag")
    for bag_path in sorted(glob.glob(pattern)):
        summary_path = os.path.join(os.path.dirname(bag_path), "run_summary.json")
        if not os.path.isfile(summary_path):
            continue
        with open(summary_path, encoding="utf-8") as stream:
            summary = json.load(stream)
        if (
            not args.force
            and "current_inflated_occupancy_samples_by_mission_state"
            in summary.get("safety", {})
        ):
            print("already enriched {}".format(summary_path))
            continue
        current_collision = False
        goal_collision = False
        emergency_stop_active = False
        emergency_stop_timeout = False
        mission_started = False
        mission_terminal = False
        mission_state = ""
        planner_samples = 0
        current_collision_samples = 0
        first_current_collision_sec = None
        last_current_collision_sec = None
        collision_samples_by_mission_state = defaultdict(int)
        with rosbag.Bag(bag_path) as bag:
            for topic, message, stamp in bag.read_messages(topics=[
                    "/uav1/tower_mission/state", "/uav1/planner/status"]):
                if topic == "/uav1/tower_mission/state":
                    mission_state = message.data
                    mission_started = mission_started or message.data != "WAIT_INPUTS"
                    mission_terminal = message.data in ("DONE", "ERROR")
                    continue
                if not mission_started or mission_terminal:
                    continue
                planner_samples += 1
                current_collision = current_collision or bool(
                    message.current_position_in_collision
                )
                goal_collision = goal_collision or bool(message.goal_in_collision)
                emergency_stop_active = (
                    emergency_stop_active or bool(message.emergency_stop_active)
                )
                emergency_stop_timeout = (
                    emergency_stop_timeout or
                    message.failure_reason == message.EMERGENCY_STOP_TIMEOUT
                )
                if message.current_position_in_collision:
                    current_collision_samples += 1
                    collision_samples_by_mission_state[
                        mission_state or "UNKNOWN"] += 1
                    sample_sec = stamp.to_sec()
                    if first_current_collision_sec is None:
                        first_current_collision_sec = sample_sec
                    last_current_collision_sec = sample_sec
        safety = summary.setdefault("safety", {})
        safety["collision"] = current_collision
        safety["collision_representation"] = (
            "PlannerStatus.current_position_in_collision against EGO inflated "
            "occupancy; physical Gazebo contact is not instrumented"
        )
        safety["goal_in_inflated_occupancy"] = goal_collision
        safety["physical_contact_collision"] = None
        safety["current_inflated_occupancy_samples"] = current_collision_samples
        safety["planner_status_samples"] = planner_samples
        safety["current_inflated_occupancy_sample_ratio"] = (
            float(current_collision_samples) / planner_samples
            if planner_samples else None
        )
        safety["first_current_inflated_occupancy_bag_time_sec"] = (
            first_current_collision_sec
        )
        safety["last_current_inflated_occupancy_bag_time_sec"] = (
            last_current_collision_sec
        )
        safety["current_inflated_occupancy_samples_by_mission_state"] = dict(
            sorted(collision_samples_by_mission_state.items())
        )
        safety["emergency_stop"] = emergency_stop_active
        planner = summary.setdefault("planner", {})
        planner["emergency_trajectory"] = emergency_stop_active
        planner["emergency_trajectory_representation"] = (
            "PlannerStatus.emergency_stop_active, i.e. EGO EMERGENCY_STOP "
            "state; no separate trajectory-class topic is exposed"
        )
        planner["emergency_stop_timeout"] = emergency_stop_timeout
        with open(summary_path, "w", encoding="utf-8") as stream:
            json.dump(summary, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print("enriched {}".format(summary_path))


if __name__ == "__main__":
    main()
