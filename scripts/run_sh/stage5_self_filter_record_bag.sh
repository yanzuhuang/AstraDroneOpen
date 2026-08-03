#!/usr/bin/env bash
set -euo pipefail

# Recorder only: it never starts PX4, arms, or publishes control commands.
output_path="${1:-/tmp/astra_stage5_self_filter_$(date +%Y%m%d-%H%M%S).bag}"
output_path="$(readlink -m "$output_path")"
mkdir -p "$(dirname "$output_path")"
if [[ -e "$output_path" || -e "${output_path}.active" ]]; then
  echo "拒绝覆盖已有 bag：$output_path" >&2
  exit 2
fi

topics=(/clock /tf /tf_static /rosout)
for uav_id in 1 2 3; do
  prefix="/uav${uav_id}"
  topics+=(
    "${prefix}/fast_lio/cloud_registered_raw"
    "${prefix}/cloud_registered"
    "${prefix}/cloud_registered_peer_filtered"
    "${prefix}/stage5/cloud_before_self"
    "${prefix}/stage5/cloud_after_self"
    "${prefix}/cloud_registered_self_filtered"
    "${prefix}/stage3/cloud_registered_filtered"
    "${prefix}/stage5/cloud_filter_diagnostics"
    "${prefix}/grid_map/occupancy"
    "${prefix}/grid_map/occupancy_inflate"
    "${prefix}/stage3/occupancy_inflate"
    "${prefix}/Odometry"
    "${prefix}/fast_lio/Odometry_raw"
    "${prefix}/planner/status"
  )
done

exec rosbag record __name:=stage5_self_filter_evidence_recorder --lz4 \
  -O "$output_path" "${topics[@]}"
