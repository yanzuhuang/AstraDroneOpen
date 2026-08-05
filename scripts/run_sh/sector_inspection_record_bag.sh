#!/usr/bin/env bash
set -euo pipefail

# Sector-inspection evidence recorder. It only records; it does not start PX4, arm, or
# publish any control command. Pass an explicit output path when needed.
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
OUT="${1:-$repo_root/runtime_artifacts/sector_inspection_record_$(date +%Y%m%d_%H%M%S)/sector_inspection.bag}"
OUT="$(readlink -m "$OUT")"
if [[ "$OUT" != "$repo_root"/runtime_artifacts/* ]]; then
  echo "rosbag output must be under $repo_root/runtime_artifacts/: $OUT" >&2
  exit 2
fi
mkdir -p "$(dirname "$OUT")"
export ROS_LOG_DIR="$(dirname "$OUT")/ros_logs"
mkdir -p "$ROS_LOG_DIR"
if [[ -e "$OUT" || -e "${OUT}.active" ]]; then
  echo "拒绝覆盖已有 bag：$OUT" >&2
  exit 2
fi
TOPICS=(
  /clock
  /livox/lidar
  /cloud_registered
  /stage2/cloud_registered_filtered
  /stage3/cloud_registered_filtered
  /Odometry
  /mavros/local_position/odom
  /mavros/local_position/pose
  /mavros/state
  /mavros/extended_state
  /mavros/setpoint_raw/local
  /mavros/setpoint_position/local
  /mavros/setpoint_velocity/cmd_vel
  /mavros/setpoint_attitude/attitude
  /mavros/setpoint_attitude/thrust
  /grid_map/occupancy
  /grid_map/occupancy_inflate
  /stage3/occupancy_inflate
  /planning/goal
  /planning/bspline
  /planning/pos_cmd
  /planning/cancel
  /ego_planner_node/goal_point
  /planner/status
  /ego_mavros_bridge/state
  /ego_mavros_bridge/tracking_error
  /tower_mission/state
  /tower_mission/current_target
  /tower_mission/current_sector
  /tower_mission/candidate_targets
  /tower_mission/progress
  /tower_mission/face_tower
  /tower_mission/selected_tower_center
  /tf
  /tf_static
  /rosout
)

exec rosbag record __name:=sector_inspection_evidence_recorder --lz4 -O "${OUT}" \
  "${TOPICS[@]}"
