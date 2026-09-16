#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../../.." && pwd)"
px4_root="${ASTRA_PX4_ROOT:-/home/yanzu/PX4-Autopilot}"
exploration_root="$repo_root/runtime_artifacts/high_speed_exploration"

enable_control=false
fixed_v_max=""
run_id=""
wall_timeout=3600
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
      echo "high_speed_exploration.sh [--control] --v-max LEVEL --run-id v050|v100|v200|v200_retry|v150 [--wall-timeout SEC]"
      echo "Default mode is non-controlling; the batch wrapper enforces formal-run order and early-stop."
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

case "$fixed_v_max:$run_id" in
  0.50:v050|1.00:v100|2.00:v200|2.00:v200_retry|1.50:v150) ;;
  *)
    echo "speed/run-id must be an audited pair: 0.50:v050, 1.00:v100, 2.00:v200, 2.00:v200_retry, 1.50:v150" >&2
    exit 2
    ;;
esac
if [[ ! "$wall_timeout" =~ ^[1-9][0-9]*$ ]]; then
  echo "--wall-timeout must be a positive integer" >&2
  exit 2
fi

run_dir="$exploration_root/$run_id"
if [[ -e "$run_dir" ]]; then
  echo "refusing to overwrite existing high-speed run: $run_dir" >&2
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
  echo "exploration_ceiling_mps=2.0 max_acc_mps2=0.50 planning_horizon_m=7.5"
  echo "unchanged_safety=clearance,inflation,collision,emergency_stop,minimum_height,occupancy,entry_exit,waypoints,observation_gates,mission_failure"
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
    python3 "$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/summarize_high_speed_exploration.py" \
      "$exploration_root" >>"$run_dir/run_metadata.txt" 2>&1 || true
  fi
  exit "$status"
}
trap cleanup EXIT INT TERM

roslaunch learning_speed_rl high_speed_exploration_uav1.launch \
  "enable_control:=$enable_control" "fixed_v_max:=$fixed_v_max" \
  exploration_ceiling:=2.0 max_acc:=0.50 \
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
      rostopic info /uav1/learning_speed/observation_c >/dev/null 2>&1 && \
      rosparam get /uav1/drone_0_ego_planner_node/manager/max_vel >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep 0.5
done
if ! "$ready"; then
  echo "high-speed topics and parameters did not become ready" >>"$run_dir/run_metadata.txt"
  exit 1
fi

read_scalar() {
  local topic="$1"
  timeout 5 rostopic echo -n 1 "$topic" 2>/dev/null | awk '/data:/ {print $2; exit}'
}

near() {
  awk -v actual="$1" -v expected="$2" \
    'BEGIN {delta=actual-expected; if (delta<0) delta=-delta; exit !(delta<=0.005)}'
}

chain_ready=false
for _ in {1..60}; do
  requested="$(read_scalar /uav1/learning_speed/raw_v_max || true)"
  filtered="$(read_scalar /uav1/learning_speed/v_max || true)"
  applied="$(read_scalar /uav1/learning_speed/applied_v_max || true)"
  if [[ -n "$requested" && -n "$filtered" && -n "$applied" ]] && \
      near "$requested" "$fixed_v_max" && near "$filtered" "$fixed_v_max" && \
      near "$applied" "$fixed_v_max"; then
    chain_ready=true
    break
  fi
  sleep 1
done

manager_ceiling="$(rosparam get /uav1/drone_0_ego_planner_node/manager/max_vel)"
optimizer_ceiling="$(rosparam get /uav1/drone_0_ego_planner_node/optimization/max_vel)"
bspline_ceiling="$(rosparam get /uav1/drone_0_ego_planner_node/bspline/limit_vel)"
dynamic_ceiling="$(rosparam get /uav1/drone_0_ego_planner_node/dynamic_speed_limit/maximum)"
filter_ceiling="$(rosparam get /uav1/speed_adapter/safety/v_max_max)"
policy_value="$(rosparam get /uav1/speed_adapter/policy/fixed_v_max)"
manager_acc="$(rosparam get /uav1/drone_0_ego_planner_node/manager/max_acc)"
optimizer_acc="$(rosparam get /uav1/drone_0_ego_planner_node/optimization/max_acc)"
bspline_acc="$(rosparam get /uav1/drone_0_ego_planner_node/bspline/limit_acc)"

parameter_chain_valid=true
for value in "$manager_ceiling" "$optimizer_ceiling" "$bspline_ceiling" \
    "$dynamic_ceiling" "$filter_ceiling"; do
  if ! near "$value" 2.0; then parameter_chain_valid=false; fi
done
if ! near "$policy_value" "$fixed_v_max"; then parameter_chain_valid=false; fi
for value in "$manager_acc" "$optimizer_acc" "$bspline_acc"; do
  if ! near "$value" 0.50; then parameter_chain_valid=false; fi
done

printf '{\n  "requested_v_max_mps": %s,\n  "filtered_v_max_mps": %s,\n  "applied_v_max_mps": %s,\n  "ego_current_limit_mps": %s,\n  "manager_ceiling_mps": %s,\n  "optimizer_ceiling_mps": %s,\n  "bspline_ceiling_mps": %s,\n  "dynamic_speed_limit_maximum_mps": %s,\n  "speed_safety_filter_maximum_mps": %s,\n  "fixed_speed_policy_mps": %s,\n  "manager_max_acc_mps2": %s,\n  "optimizer_max_acc_mps2": %s,\n  "bspline_max_acc_mps2": %s,\n  "topic_chain_valid": %s,\n  "parameter_chain_valid": %s\n}\n' \
  "${requested:-null}" "${filtered:-null}" "${applied:-null}" "${applied:-null}" \
  "$manager_ceiling" "$optimizer_ceiling" "$bspline_ceiling" \
  "$dynamic_ceiling" "$filter_ceiling" "$policy_value" \
  "$manager_acc" "$optimizer_acc" "$bspline_acc" \
  "$chain_ready" "$parameter_chain_valid" >"$run_dir/speed_chain_preflight.json"

if ! "$chain_ready" || ! "$parameter_chain_valid"; then
  echo "invalid high-speed chain; run must not be counted" >>"$run_dir/run_metadata.txt"
  exit 1
fi
echo "speed_chain_preflight=PASS" >>"$run_dir/run_metadata.txt"

rosbag record --lz4 -O "$run_dir/high_speed.bag" \
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
    if [[ -f "$run_dir/run_summary.json" ]] && \
        rg -q '"done"[[:space:]]*:[[:space:]]*true' "$run_dir/run_summary.json"; then
      terminal=true
      break
    fi
    if ((launch_status == 0)); then launch_status=1; fi
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
if ! "$terminal"; then exit 1; fi
echo "terminal_mission_done=true" >>"$run_dir/run_metadata.txt"
