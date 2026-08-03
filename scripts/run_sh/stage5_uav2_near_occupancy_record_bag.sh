#!/usr/bin/env bash
set -euo pipefail

# Recorder only: it never starts Gazebo/PX4, arms a vehicle, or publishes a
# control/mission message.  Start it after the explicitly no-control U2 stack
# is healthy.  The output must be new evidence; refusing an existing path is
# intentional.
output_path="${1:-/tmp/astra_stage5_uav2_near_occupancy_$(date +%Y%m%d-%H%M%S).bag}"
output_path="$(readlink -m "$output_path")"
mkdir -p "$(dirname "$output_path")"
if [[ -e "$output_path" || -e "${output_path}.active" ]]; then
  echo "Refusing to overwrite existing evidence bag: $output_path" >&2
  exit 2
fi

topics=(
  /clock /tf /tf_static /rosout /diagnostics
  /gazebo/model_states /gazebo/link_states
  /uav2/livox/lidar /uav2/livox/imu
  /uav2/mavros/imu/data /uav2/mavros/imu/data_raw
  /uav2/fast_lio/Odometry_raw
  /uav2/fast_lio/cloud_registered_raw
  /uav2/fast_lio/cloud_registered_body_raw
  /uav2/fast_lio/Laser_map_raw
  /uav2/fast_lio/tf_raw
  /uav2/Odometry
  /uav2/cloud_registered
  /uav2/cloud_registered_peer_filtered
  /uav2/stage5/cloud_before_self
  /uav2/stage5/cloud_after_self
  /uav2/cloud_registered_self_filtered
  /uav2/stage3/cloud_registered_filtered
  /uav2/stage5/cloud_filter_diagnostics
  /uav2/grid_map/occupancy
  /uav2/grid_map/occupancy_inflate
  /uav2/stage3/occupancy_inflate
  /uav2/mavros/local_position/pose
  /uav2/mavros/local_position/pose_framed
  /uav2/mavros/local_position/odom
  /uav2/mavros/odometry/out
  /uav2/mavros/state
  /uav2/planner/status
  /uav2/move_base_simple/goal
  /uav2/planning/goal
  /uav2/planning/bspline
  /uav2/planning/pos_cmd
  /uav2/tower_mission/state
  /uav2/tower_mission/current_target
  /uav2/tower_mission/current_sector
  /uav2/tower_mission/candidate_id
  /uav2/tower_mission/candidate_targets
  /uav2/tower_mission/entry_corridor_candidates
  /uav2/tower_mission/progress
  /uav2/ego_mavros_bridge/state
  /uav2/ego_mavros_bridge/input_health
  /uav2/ego_mavros_bridge/tracking_error
  /uav2/swarm/state /uav2/swarm/orbit_speed_scale
)

exec rosbag record __name:=stage5_uav2_near_occupancy_recorder --lz4 \
  -O "$output_path" "${topics[@]}"
