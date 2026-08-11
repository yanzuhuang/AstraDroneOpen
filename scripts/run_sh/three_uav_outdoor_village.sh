#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
px4_root="/home/yanzu/PX4-Autopilot"

gui=false
rviz=false
duration_seconds=0
while (($#)); do
  case "$1" in
    --gui) gui=true ;;
    --rviz) rviz=true ;;
    --duration)
      shift
      if (($# == 0)) || [[ ! "$1" =~ ^[1-9][0-9]*$ ]]; then
        echo "--duration requires a positive number of wall-clock seconds" >&2
        exit 2
      fi
      duration_seconds="$1"
      ;;
    --control)
      echo "--control is intentionally unavailable: this entry point never arms or flies" >&2
      exit 2
      ;;
    --help)
      echo "three_uav_outdoor_village.sh [--gui] [--rviz] [--duration SEC]"
      echo "Starts the three-UAV outdoor_village environment with control and missions hard-disabled."
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

results_dir="$repo_root/runtime_artifacts/three_uav_outdoor_village_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$results_dir/ros_home" "$results_dir/ros_logs"

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$repo_root/simulation/sim_workspace/src:$px4_root:$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export GAZEBO_MODEL_PATH="$repo_root/simulation/astra_gazebo_models:$repo_root/simulation/px4_sim_files/px4_iris_sdf:${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="$repo_root/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="$repo_root/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export ROS_HOME="$results_dir/ros_home"
export ROS_LOG_DIR="$results_dir/ros_logs"
export DISABLE_ROS1_EOL_WARNINGS=1

{
  date --iso-8601=seconds
  git -C "$repo_root" branch --show-current
  git -C "$repo_root" rev-parse HEAD
  git -C "$repo_root" status --short --branch
  echo "world=$repo_root/simulation/astra_gazebo_worlds/outdoor_village.world"
  echo "uav1=(-14.0,0.0,0.06,yaw=0.0)"
  echo "uav2=(-7.0,0.0,0.06,yaw=0.0)"
  echo "uav3=(0.0,0.0,0.06,yaw=0.0)"
  echo "enable_control=false mission_enabled=false gui=$gui rviz=$rviz duration_seconds=$duration_seconds"
} >"$results_dir/run_metadata.txt"

launch_pid=""
watchdog_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$watchdog_pid" ]] && kill -0 "$watchdog_pid" 2>/dev/null; then
    kill "$watchdog_pid" 2>/dev/null || true
    wait "$watchdog_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

roslaunch astra_swarm_bringup triple_outdoor_village_initialization.launch \
  "gui:=$gui" "start_rviz:=$rviz" \
  >"$results_dir/roslaunch.log" 2>&1 &
launch_pid=$!

if ((duration_seconds > 0)); then
  runner_pid=$$
  (
    sleep "$duration_seconds"
    echo "Initialization duration reached after ${duration_seconds}s; requesting orderly shutdown" \
      >>"$results_dir/run_metadata.txt"
    kill -INT "$runner_pid"
  ) &
  watchdog_pid=$!
fi

echo "Three-UAV outdoor_village initialization started; artifacts: $results_dir"
wait "$launch_pid"
