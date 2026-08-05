#!/usr/bin/env bash
set -euo pipefail

# Dry-run-only PlannerStatus fault injection. This script publishes status
# messages; it never calls arming, mode, takeoff, return or landing services.
# Start sector_inspection.launch with enable_control:=false first.
SCENARIO="${1:-goal_occupancy}"
RATE="${RATE:-2}"
case "${SCENARIO}" in
  goal_occupancy)
    MSG="{header: {frame_id: camera_init}, planner_state: GOAL_IN_OCCUPANCY, target_id: injected_goal, trajectory_id: 1, last_plan_success: false, consecutive_plan_failures: 1, goal_in_collision: true, current_position_in_collision: false, emergency_stop_active: false, emergency_stop_duration: 0.0, failure_reason: GOAL_IN_OCCUPANCY}"
    ;;
  current_occupancy)
    MSG="{header: {frame_id: camera_init}, planner_state: EMERGENCY_STOP, target_id: injected_goal, trajectory_id: 1, last_plan_success: false, consecutive_plan_failures: 3, goal_in_collision: false, current_position_in_collision: true, emergency_stop_active: true, emergency_stop_duration: 1.0, failure_reason: CURRENT_POSITION_IN_OCCUPANCY}"
    ;;
  replan_failed)
    MSG="{header: {frame_id: camera_init}, planner_state: REPLAN_TRAJ, target_id: injected_goal, trajectory_id: 1, last_plan_success: false, consecutive_plan_failures: 3, goal_in_collision: false, current_position_in_collision: false, emergency_stop_active: false, emergency_stop_duration: 0.0, failure_reason: REPLAN_FAILED}"
    ;;
  emergency_stop_timeout)
    MSG="{header: {frame_id: camera_init}, planner_state: EMERGENCY_STOP, target_id: injected_goal, trajectory_id: 1, last_plan_success: false, consecutive_plan_failures: 4, goal_in_collision: false, current_position_in_collision: false, emergency_stop_active: true, emergency_stop_duration: 10.0, failure_reason: EMERGENCY_STOP_TIMEOUT}"
    ;;
  map_stale)
    MSG="{header: {frame_id: camera_init}, planner_state: MAP_STALE, target_id: injected_goal, trajectory_id: 1, last_plan_success: false, consecutive_plan_failures: 0, goal_in_collision: false, current_position_in_collision: false, emergency_stop_active: false, emergency_stop_duration: 0.0, failure_reason: MAP_STALE}"
    ;;
  *)
    echo "usage: $0 {goal_occupancy|current_occupancy|replan_failed|emergency_stop_timeout|map_stale}" >&2
    exit 2
    ;;
esac

exec rostopic pub -r "${RATE}" /planner/status astra_custom_msgs/PlannerStatus "${MSG}"
