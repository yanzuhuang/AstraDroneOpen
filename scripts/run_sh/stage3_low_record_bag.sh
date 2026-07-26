#!/usr/bin/env bash
set -Eeuo pipefail

output="${1:?需要 rosbag 输出路径}"
output="$(readlink -m "$output")"
mkdir -p "$(dirname "$output")"
if [[ -e "$output" || -e "${output}.active" ]]; then
    echo "拒绝覆盖已有 bag：$output" >&2
    exit 2
fi

topics=(
    /clock
    /stage3_low/cloud_registered_filtered
    /Odometry
    /mavros/local_position/odom
    /mavros/local_position/pose
    /mavros/state
    /mavros/extended_state
    /mavros/setpoint_raw/local
    /gazebo/model_states
    /grid_map/occupancy_inflate
    /stage3_low/occupancy_inflate
    /planning/goal
    /planning/bspline
    /planning/pos_cmd
    /planning/cancel
    /planner/status
    /ego_mavros_bridge/state
    /ego_mavros_bridge/tracking_error
    /tower_mission/state
    /tower_mission/current_target
    /tower_mission/current_sector
    /tower_mission/progress
    /tower_mission/face_tower
    /tower_mission/selected_tower_center
    /tower_mission/level_ingress_path
    /tower_mission/level_orbit_path
    /tower_mission/altitude_policy
    /tf
    /tf_static
    /rosout
)

exec rosbag record __name:=stage3_low_evidence_recorder --lz4 \
    -O "$output" "${topics[@]}"
