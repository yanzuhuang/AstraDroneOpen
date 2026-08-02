#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
px4_root="/home/yanzu/PX4-Autopilot"

enable_control=false
gui=false
rviz=false
results_dir=""
duration_seconds=0
while (($#)); do
  case "$1" in
    --control) enable_control=true ;;
    --gui) gui=true ;;
    --rviz) rviz=true ;;
    --results-dir)
      shift
      if (($# == 0)); then
        echo "--results-dir requires a path" >&2
        exit 2
      fi
      results_dir="$1"
      ;;
    --duration)
      shift
      if (($# == 0)) || [[ ! "$1" =~ ^[1-9][0-9]*$ ]]; then
        echo "--duration requires a positive number of wall-clock seconds" >&2
        exit 2
      fi
      duration_seconds="$1"
      ;;
    --help)
      echo "stage5_three_uav.sh [--control] [--gui] [--rviz] [--duration SEC] [--results-dir DIR]"
      echo "Every run stores ROS logs, CSV/JSONL, summary, console output and a rosbag under test_evidence/."
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

mode="dry_run"
if "$enable_control"; then
  mode="control"
fi
if [[ -z "$results_dir" ]]; then
  results_dir="$repo_root/test_evidence/stage5_${mode}_$(date +%Y%m%d_%H%M%S)"
fi
if [[ "$results_dir" != "$repo_root"/test_evidence/stage5_* ]]; then
  echo "results directory must be a timestamped test_evidence/stage5_* path" >&2
  exit 2
fi
mkdir -p "$results_dir/ros_logs"

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$repo_root/simulation/sim_workspace/src:$px4_root:$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export GAZEBO_MODEL_PATH="$repo_root/simulation/astra_gazebo_models:$repo_root/simulation/px4_sim_files/px4_iris_sdf:${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="$repo_root/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="$repo_root/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export ROS_LOG_DIR="$results_dir/ros_logs"

{
  date --iso-8601=seconds
  git -C "$repo_root" branch --show-current
  git -C "$repo_root" rev-parse HEAD
  git -C "$repo_root" status --short --branch
  echo "mode=$mode gui=$gui rviz=$rviz duration_seconds=$duration_seconds"
} >"$results_dir/run_metadata.txt"

launch_pid=""
bag_pid=""
watchdog_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$watchdog_pid" ]] && kill -0 "$watchdog_pid" 2>/dev/null; then
    kill "$watchdog_pid" 2>/dev/null || true
    wait "$watchdog_pid" 2>/dev/null || true
  fi
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ -s "$results_dir/swarm.csv" ]]; then
    python3 "$repo_root/scripts/tool/plot_stage5_trajectory.py" \
      "$results_dir/swarm.csv" "$results_dir/trajectory_xy.png" \
      --output-3d "$results_dir/trajectory_3d.png" \
      >>"$results_dir/run_metadata.txt" 2>&1 || \
      echo "trajectory plot generation failed" \
        >>"$results_dir/run_metadata.txt"
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

roslaunch astra_swarm_bringup triple_tower_inspection.launch \
  enable_control:="$enable_control" gui:="$gui" start_rviz:="$rviz" \
  uav1_report_file:="$results_dir/uav1.csv" \
  uav2_report_file:="$results_dir/uav2.csv" \
  uav3_report_file:="$results_dir/uav3.csv" \
  evidence_csv_file:="$results_dir/swarm.csv" \
  evidence_summary_file:="$results_dir/summary.json" \
  evidence_candidate_file:="$results_dir/candidates.jsonl" \
  >"$results_dir/roslaunch.log" 2>&1 &
launch_pid=$!

for _ in {1..120}; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid"
  fi
  if rosparam get /use_sim_time >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

topics=(/clock /rosout /diagnostics /gazebo/model_states
  /swarm/state /swarm/safety/clear
  /swarm/safety/event /swarm/coordinator/status /swarm/formation/status
  /swarm/entry_corridor_audit /swarm/trajectories)
for uid in 1 2 3; do
  prefix="/uav${uid}"
  topics+=(
    "$prefix/mavros/local_position/pose"
    "$prefix/mavros/local_position/pose_framed"
    "$prefix/mavros/state"
    "$prefix/mavros/extended_state"
    "$prefix/mavros/timesync_status"
    "$prefix/fast_lio/Odometry_raw"
    "$prefix/fast_lio/cloud_registered_raw"
    "$prefix/Odometry"
    "$prefix/cloud_registered"
    "$prefix/stage3/occupancy_inflate"
    "$prefix/swarm/state"
    "$prefix/swarm/takeoff_permission"
    "$prefix/swarm/orbit_permission"
    "$prefix/swarm/orbit_speed_scale"
    "$prefix/swarm/entry_corridor_selection"
    "$prefix/ego_mavros_bridge/state"
    "$prefix/ego_mavros_bridge/input_health"
    "$prefix/ego_mavros_bridge/tracking_error"
    "$prefix/move_base_simple/goal"
    "$prefix/planning/goal"
    "$prefix/planning/bspline"
    "$prefix/planning/pos_cmd"
    "$prefix/planner/status"
    "$prefix/tower_mission/state"
    "$prefix/tower_mission/current_target"
    "$prefix/tower_mission/current_sector"
    "$prefix/tower_mission/candidate_targets"
    "$prefix/tower_mission/entry_corridor_candidates"
    "$prefix/tower_mission/progress"
  )
done
rosbag record --lz4 -O "$results_dir/stage5.bag" "${topics[@]}" \
  >"$results_dir/rosbag.log" 2>&1 &
bag_pid=$!

if ((duration_seconds > 0)); then
  runner_pid=$$
  (
    sleep "$duration_seconds"
    echo "Stage 5 $mode duration reached after ${duration_seconds}s; requesting orderly shutdown" \
      >>"$results_dir/run_metadata.txt"
    kill -INT "$runner_pid"
  ) &
  watchdog_pid=$!
fi

echo "Stage 5 $mode started; permanent evidence: $results_dir"
wait "$launch_pid"
