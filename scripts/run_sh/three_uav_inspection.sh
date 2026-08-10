#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
px4_root="/home/yanzu/PX4-Autopilot"

enable_control=false
gui=false
rviz=false
record_mode="light"
results_dir=""
results_dir_requested=false
duration_seconds=0
while (($#)); do
  case "$1" in
    --control) enable_control=true ;;
    --gui) gui=true ;;
    --rviz) rviz=true ;;
    --record)
      shift
      if (($# == 0)); then
        echo "--record requires one of: none, light, full" >&2
        exit 2
      fi
      case "$1" in
        none|light|full) record_mode="$1" ;;
        *)
          echo "--record requires one of: none, light, full" >&2
          exit 2
          ;;
      esac
      ;;
    --results-dir)
      shift
      if (($# == 0)); then
        echo "--results-dir requires a path" >&2
        exit 2
      fi
      results_dir="$1"
      results_dir_requested=true
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
      echo "three_uav_inspection.sh [--control] [--gui] [--rviz] [--record none|light|full] [--duration SEC] [--results-dir DIR]"
      echo "The default --record light mode stores state and trajectory evidence without point clouds or Gazebo model states."
      echo "Use --record none to retain no task evidence, bag, CSV, summary, launch log or trajectory plots."
      echo "Light and full modes save trajectory_xy.png and trajectory_3d.png when swarm.csv contains samples."
      echo "Use --results-dir to select a timestamped runtime_artifacts/ directory for light or full mode."
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
if [[ "$record_mode" == none ]] && "$results_dir_requested"; then
  echo "--results-dir cannot be used with --record none because none retains no artifacts" >&2
  exit 2
fi
temporary_runtime_dir=""
cleanup_temporary_runtime_dir() {
  if [[ -z "$temporary_runtime_dir" ]]; then
    return
  fi
  case "$temporary_runtime_dir" in
    /tmp/astra_three_uav_none.*)
      rm -rf -- "$temporary_runtime_dir"
      ;;
    *)
      echo "refusing to remove unexpected temporary directory: $temporary_runtime_dir" >&2
      ;;
  esac
}
if [[ "$record_mode" == none ]]; then
  temporary_runtime_dir="$(mktemp -d /tmp/astra_three_uav_none.XXXXXX)"
  trap cleanup_temporary_runtime_dir EXIT
  mkdir -p "$temporary_runtime_dir/ros_logs"
else
  if [[ -z "$results_dir" ]]; then
    results_dir="$repo_root/runtime_artifacts/three_uav_inspection_${mode}_$(date +%Y%m%d_%H%M%S)"
  fi
  if [[ "$results_dir" != "$repo_root"/runtime_artifacts/three_uav_inspection_* ]]; then
    echo "results directory must be a timestamped runtime_artifacts/three_uav_inspection_* path" >&2
    exit 2
  fi
  mkdir -p "$results_dir/ros_logs"
fi

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$repo_root/simulation/sim_workspace/src:$px4_root:$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export GAZEBO_MODEL_PATH="$repo_root/simulation/astra_gazebo_models:$repo_root/simulation/px4_sim_files/px4_iris_sdf:${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="$repo_root/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="$repo_root/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
if [[ "$record_mode" == none ]]; then
  export ROS_LOG_DIR="$temporary_runtime_dir/ros_logs"
  export ROSOUT_DISABLE_FILE_LOGGING=True
else
  export ROS_LOG_DIR="$results_dir/ros_logs"
  {
    date --iso-8601=seconds
    git -C "$repo_root" branch --show-current
    git -C "$repo_root" rev-parse HEAD
    git -C "$repo_root" status --short --branch
    echo "mode=$mode gui=$gui rviz=$rviz record_mode=$record_mode duration_seconds=$duration_seconds"
  } >"$results_dir/run_metadata.txt"
fi

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
  if [[ "$record_mode" != none && -s "$results_dir/swarm.csv" ]]; then
    python3 "$repo_root/scripts/tool/plot_three_uav_trajectory.py" \
      "$results_dir/swarm.csv" "$results_dir/trajectory_xy.png" \
      --output-3d "$results_dir/trajectory_3d.png" \
      >>"$results_dir/run_metadata.txt" 2>&1 || \
      echo "trajectory plot generation failed" \
        >>"$results_dir/run_metadata.txt"
  fi
  cleanup_temporary_runtime_dir
  exit "$status"
}
trap cleanup EXIT INT TERM

uav1_report_file="/dev/null"
uav2_report_file="/dev/null"
uav3_report_file="/dev/null"
evidence_record_enabled=false
if [[ "$record_mode" != none ]]; then
  uav1_report_file="$results_dir/uav1.csv"
  uav2_report_file="$results_dir/uav2.csv"
  uav3_report_file="$results_dir/uav3.csv"
  evidence_record_enabled=true
fi
launch_args=(astra_swarm_bringup triple_tower_inspection.launch
  "enable_control:=$enable_control" "gui:=$gui" "start_rviz:=$rviz"
  "uav1_report_file:=$uav1_report_file"
  "uav2_report_file:=$uav2_report_file"
  "uav3_report_file:=$uav3_report_file"
  "evidence_record_enabled:=$evidence_record_enabled"
  "evidence_candidate_record_mode:=$record_mode")
if [[ "$record_mode" == none ]]; then
  roslaunch "${launch_args[@]}" &
else
  launch_args+=(
    "evidence_csv_file:=$results_dir/swarm.csv"
    "evidence_summary_file:=$results_dir/summary.json"
    "evidence_candidate_file:=$results_dir/candidates.jsonl")
  roslaunch "${launch_args[@]}" >"$results_dir/roslaunch.log" 2>&1 &
fi
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

topics=(/clock /rosout /diagnostics
  /swarm/state /swarm/safety/clear
  /swarm/safety/event /swarm/coordinator/status /swarm/formation/status
  /swarm/entry_corridor_audit /swarm/trajectories)
if [[ "$record_mode" == full ]]; then
  topics+=(/gazebo/model_states)
fi
for uid in 1 2 3; do
  prefix="/uav${uid}"
  topics+=(
    "$prefix/mavros/local_position/pose_framed"
    "$prefix/mavros/state"
    "$prefix/mavros/extended_state"
    "$prefix/mavros/timesync_status"
    "$prefix/Odometry"
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
    "$prefix/tower_mission/progress"
  )
  if [[ "$record_mode" == full ]]; then
    topics+=(
      "$prefix/mavros/local_position/pose"
      "$prefix/fast_lio/Odometry_raw"
      "$prefix/fast_lio/cloud_registered_raw"
      "$prefix/cloud_registered"
      "$prefix/stage3/occupancy_inflate"
      "$prefix/tower_mission/candidate_targets"
      "$prefix/tower_mission/entry_corridor_candidates"
    )
  fi
done
if [[ "$record_mode" != none ]]; then
  rosbag record --lz4 -O "$results_dir/three_uav_inspection.bag" \
    "${topics[@]}" >"$results_dir/rosbag.log" 2>&1 &
  bag_pid=$!
fi

if ((duration_seconds > 0)); then
  runner_pid=$$
  (
    sleep "$duration_seconds"
    if [[ "$record_mode" == none ]]; then
      echo "Three-UAV inspection $mode duration reached after ${duration_seconds}s; requesting orderly shutdown"
    else
      echo "Three-UAV inspection $mode duration reached after ${duration_seconds}s; requesting orderly shutdown" \
        >>"$results_dir/run_metadata.txt"
    fi
    kill -INT "$runner_pid"
  ) &
  watchdog_pid=$!
fi

if [[ "$record_mode" == none ]]; then
  echo "Three-UAV inspection $mode started; record=none; no task artifacts will be retained"
else
  echo "Three-UAV inspection $mode started; record=$record_mode; artifacts: $results_dir"
fi
wait "$launch_pid"
