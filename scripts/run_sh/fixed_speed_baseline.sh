#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
px4_root="${ASTRA_PX4_ROOT:-/home/yanzu/PX4-Autopilot}"
baseline_root="$repo_root/runtime_artifacts/fixed_speed_baseline"

enable_control=false
fixed_v_max=""
run_id=""
wall_timeout=2400
while (($#)); do
  case "$1" in
    --control) enable_control=true ;;
    --v-max)
      shift
      fixed_v_max="${1:-}"
      ;;
    --run-id)
      shift
      run_id="${1:-}"
      ;;
    --wall-timeout)
      shift
      wall_timeout="${1:-}"
      ;;
    --help)
      echo "fixed_speed_baseline.sh [--control] --v-max 0.08|0.12|0.16|0.20 --run-id ID [--wall-timeout SEC]"
      echo "Default mode is launch/record preview; --control is required for arm/takeoff."
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

case "$fixed_v_max" in
  0.08|0.12|0.16|0.20) ;;
  *)
    echo "--v-max must be one of the audited baseline levels: 0.08, 0.12, 0.16, 0.20" >&2
    exit 2
    ;;
esac
if [[ ! "$run_id" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]]; then
  echo "--run-id must contain only letters, digits, dot, underscore or dash" >&2
  exit 2
fi
if [[ ! "$wall_timeout" =~ ^[1-9][0-9]*$ ]]; then
  echo "--wall-timeout must be a positive integer" >&2
  exit 2
fi

run_dir="$baseline_root/runs/$run_id"
if [[ -e "$run_dir" ]]; then
  echo "refusing to overwrite existing baseline run: $run_dir" >&2
  exit 2
fi
mkdir -p "$run_dir/ros_logs"

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$repo_root/simulation/sim_workspace/src:$px4_root:$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export GAZEBO_MODEL_PATH="$repo_root/simulation/astra_gazebo_models:$repo_root/simulation/px4_sim_files/px4_iris_sdf:${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="$repo_root/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="$repo_root/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export ROS_LOG_DIR="$run_dir/ros_logs"

{
  date --iso-8601=seconds
  git -C "$repo_root" branch --show-current
  git -C "$repo_root" rev-parse HEAD
  git -C "$repo_root" status --short --branch
  echo "run_id=$run_id fixed_v_max_mps=$fixed_v_max enable_control=$enable_control wall_timeout_sec=$wall_timeout"
  echo "world=$repo_root/simulation/astra_gazebo_worlds/worksite.world"
  echo "static_ego_max_vel_mps=0.20 safety_filter_range_mps=[0.05,0.20]"
  echo "randomness=Gazebo_physics_scheduler_sensor_timing_and_any_plugin_noise; no per-run seed interface is currently exposed"
} >"$run_dir/run_metadata.txt"

launch_pid=""
bag_pid=""
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$bag_pid" ]] && kill -0 "$bag_pid" 2>/dev/null; then
    kill -INT "$bag_pid" 2>/dev/null || true
    wait "$bag_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  if [[ -f "$run_dir/run_summary.json" ]]; then
    python3 "$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/enrich_fixed_speed_baseline.py" \
      "$baseline_root" >>"$run_dir/run_metadata.txt" 2>&1 || true
    python3 "$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/summarize_fixed_speed_baseline.py" \
      "$baseline_root" >>"$run_dir/run_metadata.txt" 2>&1 || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

roslaunch learning_speed_rl fixed_speed_baseline_uav1.launch \
  "enable_control:=$enable_control" "fixed_v_max:=$fixed_v_max" \
  "run_id:=$run_id" "output_dir:=$run_dir" \
  "world:=$repo_root/simulation/astra_gazebo_worlds/worksite.world" \
  d435_enabled:=true lidar_downsample:=1 \
  >"$run_dir/roslaunch.log" 2>&1 &
launch_pid=$!

ready=false
for _ in {1..240}; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid"
  fi
  if rostopic info /uav1/tower_mission/mission_done >/dev/null 2>&1 && \
      rostopic info /uav1/learning_speed/observation_c >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep 0.5
done
if ! "$ready"; then
  echo "baseline topics did not become ready" >>"$run_dir/run_metadata.txt"
  exit 1
fi

rosbag record --lz4 -O "$run_dir/baseline.bag" \
  /clock /rosout /diagnostics \
  /uav1/Odometry /uav1/planning/bspline /uav1/planner/status \
  /uav1/tower_mission/state /uav1/tower_mission/current_target \
  /uav1/tower_mission/mission_success /uav1/tower_mission/mission_failure \
  /uav1/tower_mission/mission_done /uav1/tower_mission/orbit_complete \
  /uav1/learning_speed/raw_v_max /uav1/learning_speed/v_max \
  /uav1/learning_speed/applied_v_max \
  /uav1/learning_speed/observation_c \
  /uav1/learning_speed/observation_c/valid \
  /uav1/learning_speed/observation_c/diagnostics \
  /uav1/mavros/state /uav1/mavros/extended_state \
  >"$run_dir/rosbag.log" 2>&1 &
bag_pid=$!

terminal=false
start_epoch="$(date +%s)"
while true; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    launch_status=0
    wait "$launch_pid" || launch_status=$?
    launch_pid=""
    # A required mission/recorder node may let roslaunch exit immediately
    # after writing its latched terminal artifact.  Treat the immutable JSON
    # as authoritative instead of misclassifying that teardown race as an
    # infrastructure interruption.
    if [[ -f "$run_dir/run_summary.json" ]] &&
        rg -q '"done"[[:space:]]*:[[:space:]]*true' \
          "$run_dir/run_summary.json"; then
      terminal=true
      break
    fi
    if ((launch_status == 0)); then
      launch_status=1
    fi
    exit "$launch_status"
  fi
  done_value="$(timeout 3 rostopic echo -n 1 /uav1/tower_mission/mission_done 2>/dev/null | awk '/data:/ {print $2; exit}' || true)"
  if [[ "${done_value,,}" == true ]]; then
    terminal=true
    break
  fi
  now_epoch="$(date +%s)"
  if ((now_epoch - start_epoch >= wall_timeout)); then
    echo "wall_timeout=true" >>"$run_dir/run_metadata.txt"
    break
  fi
  sleep 5
done

sleep 2
if ! "$terminal"; then
  exit 1
fi
echo "terminal_mission_done=true" >>"$run_dir/run_metadata.txt"
