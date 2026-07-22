#!/usr/bin/env bash
set -euo pipefail

# Stage3 evidence recorder. It only records; it does not start PX4, arm, or
# publish any control command. Pass an explicit output path when needed.
OUT="${1:-/tmp/astra_stage3_$(date +%Y%m%d-%H%M%S).bag}"
TOPICS=(
  /livox/lidar
  /cloud_registered
  /stage2/cloud_registered_filtered
  /stage3/cloud_registered_filtered
  /Odometry
  /mavros/local_position/odom
  /grid_map/occupancy
  /grid_map/occupancy_inflate
  /planning/goal
  /planning/bspline
  /planning/pos_cmd
  /ego_planner_node/goal_point
  /planner/status
  /ego_mavros_bridge/state
  /ego_mavros_bridge/tracking_error
  /tower_mission/state
  /tower_mission/current_target
  /tower_mission/current_sector
  /tower_mission/candidate_targets
  /tf
  /tf_static
  /rosout
)

exec rosbag record --lz4 -O "${OUT}" "${TOPICS[@]}"
