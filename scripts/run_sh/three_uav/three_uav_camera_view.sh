#!/usr/bin/env bash
# Attach image viewers to an existing simulation; never launch flight processes.
set -eo pipefail
backend=ros
case "${1:-}" in
  --help|-h)
    echo "Usage: three_uav_camera_view.sh [--gazebo]"
    echo "Open UAV1/UAV2/UAV3 D435 color windows for an already running simulation."
    echo "Requires D435 enabled. Ctrl+C closes only these viewers."
    echo "--gazebo uses native Gazebo image topics instead of ROS image_view."
    exit 0 ;;
  --gazebo) backend=gazebo; shift ;;
  "") ;;
  *) echo "Unknown argument: $1" >&2; exit 2 ;;
esac
if (( $# > 0 )); then
  echo "No positional arguments are supported." >&2
  exit 2
fi
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
source /opt/ros/noetic/setup.bash
if [[ -z "${DISPLAY:-}" ]]; then
  echo "No DISPLAY. Run this command in a desktop terminal." >&2
  exit 1
fi
if [[ "$backend" == ros ]] && ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "Cannot reach ROS master. Start the three-UAV simulation first." >&2
  exit 1
fi
viewer=/opt/ros/noetic/lib/image_view/image_view
if [[ "$backend" == ros && ! -x "$viewer" ]]; then
  echo "Missing ROS Noetic image_view executable: $viewer" >&2
  exit 1
fi
topics=()
if [[ "$backend" == gazebo ]]; then
  topic_list="$(timeout 5 gz topic -l)" || {
    echo "Cannot list Gazebo topics. Start the simulation first." >&2
    exit 1
  }
  for uav in 1 2 3; do
    mapfile -t matches < <(printf '%s\n' "$topic_list" | rg "/UAV${uav}_Camera_color/image$")
    if (( ${#matches[@]} != 1 )); then
      echo "Expected one native UAV${uav} color image topic, found ${#matches[@]}. Check D435 and Gazebo master." >&2
      exit 1
    fi
    topics+=("${matches[0]}")
  done
fi
mkdir -p "$repo_root/runtime_artifacts"
run_dir="$(mktemp -d "$repo_root/runtime_artifacts/three_uav_camera_view_$(date +%Y%m%d_%H%M%S)_XXXXXX")"
export ROS_LOG_DIR="$run_dir/ros_logs"
mkdir -p "$ROS_LOG_DIR"
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
echo "Opening three color views. Logs: $run_dir"
echo "If no images arrive, check D435 is enabled and Gazebo is unpaused. Ctrl+C closes the viewers."
for uav in 1 2 3; do
  if [[ "$backend" == gazebo ]]; then
    gz topic -v "${topics[uav-1]}" >"$run_dir/uav${uav}.log" 2>&1 &
  else
  "$viewer" "__name:=uav${uav}_fpv_$$" \
    "image:=/uav${uav}/d435/color/image_raw" \
    "_window_name:=UAV${uav} FPV" _autosize:=true \
    "_filename_format:=$run_dir/uav${uav}_%04i.jpg" \
    >"$run_dir/uav${uav}.log" 2>&1 &
  fi
  pids+=("$!")
done
for pid in "${pids[@]}"; do wait "$pid"; done
