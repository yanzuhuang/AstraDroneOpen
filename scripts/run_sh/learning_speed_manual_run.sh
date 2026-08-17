#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
px4_root="${ASTRA_PX4_ROOT:-/home/yanzu/PX4-Autopilot}"
calibration_root="$repo_root/runtime_artifacts/learning_speed/calibration"
summary_script="$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/summarize_manual_calibration.py"
task_config="$repo_root/AstraDrone_ros1_ws/src/MissionControl/astra_tower_mission/config/low_altitude_inspection.yaml"

enable_control=false
environment=""
fixed_v_max=""
logical_run_id=""
attempt=1
wall_timeout=5400
while (($#)); do
  case "$1" in
    --control) enable_control=true ;;
    --environment) shift; environment="${1:-}" ;;
    --v-max) shift; fixed_v_max="${1:-}" ;;
    --run-id) shift; logical_run_id="${1:-}" ;;
    --attempt) shift; attempt="${1:-}" ;;
    --wall-timeout) shift; wall_timeout="${1:-}" ;;
    --help)
      echo "learning_speed_manual_run.sh [--control] --environment A|B --v-max 0.30|0.50|0.75|1.00|1.25|1.50 --run-id ID [--attempt 1|2]"
      echo "Runs UAV1 with the fixed Learning Speed source and the read-only Data Contract collector."
      exit 0
      ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

case "$fixed_v_max" in
  0.30|0.50|0.75|1.00|1.25|1.50) ;;
  *) echo "--v-max must be one of 0.30, 0.50, 0.75, 1.00, 1.25, 1.50" >&2; exit 2 ;;
esac
case "$environment" in
  A)
    environment_name="outdoor_village_open_route"
    world="$repo_root/simulation/astra_gazebo_worlds/outdoor_village.world"
    spawn_x="-14.0"
    spawn_y="0.0"
    tower_center_x="-30.0"
    tower_center_y="15.0"
    crane_center_x="2.7535"
    ;;
  B)
    environment_name="worksite_tower_dense_route"
    world="$repo_root/simulation/astra_gazebo_worlds/worksite.world"
    spawn_x="0.0"
    spawn_y="0.0"
    tower_center_x="-10.0551"
    tower_center_y="19.7104"
    crane_center_x="2.7535"
    ;;
  *) echo "--environment must be A or B" >&2; exit 2 ;;
esac
if [[ ! "$logical_run_id" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]*$ ]]; then
  echo "--run-id must contain only letters, digits, dot, underscore or dash" >&2
  exit 2
fi
if [[ ! "$attempt" =~ ^[12]$ ]]; then
  echo "--attempt must be 1 or 2" >&2
  exit 2
fi
if [[ ! "$wall_timeout" =~ ^[1-9][0-9]*$ ]]; then
  echo "--wall-timeout must be a positive integer" >&2
  exit 2
fi

attempt_run_id="$logical_run_id"
if [[ "$attempt" == 2 ]]; then
  attempt_run_id="${logical_run_id}_infra_retry"
fi
run_dir="$calibration_root/$attempt_run_id"
if [[ -e "$run_dir" ]]; then
  echo "refusing to overwrite existing calibration attempt: $run_dir" >&2
  exit 2
fi
mkdir -p "$calibration_root"

if rosnode list >/dev/null 2>&1; then
  echo "an existing ROS master is reachable; refusing to mix calibration with residual nodes" >&2
  exit 20
fi

route_fingerprint="$({ sha256sum "$task_config"; printf '%s\n' \
  "uav1_only tower=($tower_center_x,$tower_center_y) height=3.0 radius=12.5 sectors=8 laps=1 entry_sector=7 entry_angle=292.5 direction=counter_clockwise speed_ceiling=1.50 max_acc=0.50 planning_horizon=7.5 overall_timeout=900.0"; } | sha256sum | awk '{print $1}')"
temp_dir="$(mktemp -d /tmp/astra_learning_speed_manual.XXXXXX)"
mkdir -p "$temp_dir/ros_home" "$temp_dir/ros_logs"
fast_lio_mat_pre="$repo_root/AstraDrone_ros1_ws/src/SLAM/FAST_LIO/Log/mat_pre.txt"
cp "$fast_lio_mat_pre" "$temp_dir/mat_pre.before_run.txt"

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH:-}:$repo_root/simulation/sim_workspace/src:$px4_root:$px4_root/Tools/simulation/gazebo-classic/sitl_gazebo-classic"
export GAZEBO_MODEL_PATH="$repo_root/simulation/astra_gazebo_models:$repo_root/simulation/px4_sim_files/px4_iris_sdf:${GAZEBO_MODEL_PATH:-}"
export LD_LIBRARY_PATH="$repo_root/simulation/sim_workspace/devel/lib:${LD_LIBRARY_PATH:-}"
export GAZEBO_PLUGIN_PATH="$repo_root/simulation/sim_workspace/devel/lib:${GAZEBO_PLUGIN_PATH:-}"
export ROS_HOME="$temp_dir/ros_home"
export ROS_LOG_DIR="$temp_dir/ros_logs"
export DISABLE_ROS1_EOL_WARNINGS=1

launch_pid=""
infrastructure_failure=false
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
  mkdir -p "$run_dir"
  if [[ -f "$temp_dir/roslaunch.log" ]]; then
    cp "$temp_dir/roslaunch.log" "$run_dir/roslaunch.log"
  fi
  if [[ -f "$temp_dir/mission.csv" ]]; then
    cp "$temp_dir/mission.csv" "$run_dir/mission.csv"
  fi
  if [[ -d "$temp_dir/ros_logs" ]]; then
    cp -a "$temp_dir/ros_logs" "$run_dir/ros_logs"
  fi
  # FAST-LIO opens this tracked debug file unconditionally.  Restore the exact
  # pre-run bytes so calibration never consumes or overwrites workspace data.
  if [[ -f "$temp_dir/mat_pre.before_run.txt" ]]; then
    cp "$temp_dir/mat_pre.before_run.txt" "$fast_lio_mat_pre"
  fi
  if [[ -f "$run_dir/experiment_manifest.json" ]] && "$infrastructure_failure"; then
    sed -i 's/"infrastructure_failure": false/"infrastructure_failure": true/' \
      "$run_dir/experiment_manifest.json"
  fi
  python3 "$summary_script" "$calibration_root" >/dev/null 2>&1 || true
  rm -rf "$temp_dir"
  exit "$status"
}
trap cleanup EXIT INT TERM

roslaunch learning_speed_rl manual_fixed_speed_calibration_uav1.launch \
  "enable_control:=$enable_control" "fixed_v_max:=$fixed_v_max" \
  "environment:=$environment" \
  speed_ceiling:=1.50 max_acc:=0.50 \
  "run_id:=$attempt_run_id" "output_dir:=$run_dir" \
  "world:=$world" "uav1_spawn_x:=$spawn_x" "spawn_y:=$spawn_y" \
  "tower_center_x:=$tower_center_x" "tower_center_y:=$tower_center_y" \
  "crane_center_x:=$crane_center_x" \
  "task_config:=$task_config" "mission_report_file:=$temp_dir/mission.csv" \
  d435_enabled:=false lidar_downsample:=1 \
  >"$temp_dir/roslaunch.log" 2>&1 &
launch_pid=$!

ready=false
for _ in {1..300}; do
  if ! kill -0 "$launch_pid" 2>/dev/null; then
    wait "$launch_pid" || true
    launch_pid=""
    break
  fi
  if [[ -f "$run_dir/calibration_samples.csv" ]] && \
      rostopic info /uav1/tower_mission/mission_done >/dev/null 2>&1 && \
      rostopic info /uav1/learning_speed/observation_c >/dev/null 2>&1 && \
      rostopic info /uav1/learning_speed/action_stamped >/dev/null 2>&1 && \
      rosparam get /uav1/drone_0_ego_planner_node/manager/max_vel >/dev/null 2>&1; then
    ready=true
    break
  fi
  sleep 0.5
done

mkdir -p "$run_dir"
printf '{\n  "schema_version": "learning_speed_manual_fixed_v1.0",\n  "logical_run_id": "%s",\n  "attempt_run_id": "%s",\n  "attempt": %s,\n  "environment": "%s",\n  "environment_name": "%s",\n  "world": "%s",\n  "spawn_x_m": %s,\n  "spawn_y_m": %s,\n  "tower_center_x_m": %s,\n  "tower_center_y_m": %s,\n  "fixed_v_max_mps": %s,\n  "speed_ceiling_mps": 1.50,\n  "max_acc_mps2": 0.50,\n  "planning_horizon_m": 7.5,\n  "mission_overall_timeout_sec": 900.0,\n  "task_config": "%s",\n  "route_fingerprint": "%s",\n  "policy_mode": "fixed",\n  "sac_enabled": false,\n  "reward_defined": false,\n  "tracking_gate_m": 1.0,\n  "tracking_gate_duration_sec": 1.0,\n  "infrastructure_failure": false\n}\n' \
  "$logical_run_id" "$attempt_run_id" "$attempt" "$environment" \
  "$environment_name" "$world" "$spawn_x" "$spawn_y" \
  "$tower_center_x" "$tower_center_y" "$fixed_v_max" \
  "$task_config" "$route_fingerprint" >"$run_dir/experiment_manifest.json"

{
  date --iso-8601=seconds
  git -C "$repo_root" branch --show-current
  git -C "$repo_root" rev-parse HEAD
  git -C "$repo_root" status --short --branch
  echo "enable_control=$enable_control policy_mode=fixed sac_enabled=false reward_defined=false"
  echo "environment=$environment world=$world spawn=($spawn_x,$spawn_y,0.06) tower_center=($tower_center_x,$tower_center_y)"
  echo "fixed_v_max_mps=$fixed_v_max speed_ceiling_mps=1.50 max_acc_mps2=0.50"
  echo "route_fingerprint=$route_fingerprint task_config=$task_config"
} >"$run_dir/run_metadata.txt"

if ! "$ready"; then
  infrastructure_failure=true
  echo "startup_preflight=FAIL" >>"$run_dir/run_metadata.txt"
  exit 20
fi

read_scalar() {
  timeout 5 rostopic echo -n 1 "$1" 2>/dev/null | awk '/data:/ {print $2; exit}'
}
near() {
  awk -v actual="$1" -v expected="$2" \
    'BEGIN {delta=actual-expected; if (delta<0) delta=-delta; exit !(delta<=0.005)}'
}

chain_ready=false
requested=""; filtered=""; applied=""
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
policy_mode="$(rosparam get /uav1/speed_adapter/policy/mode)"
policy_value="$(rosparam get /uav1/speed_adapter/policy/fixed_v_max)"
manager_acc="$(rosparam get /uav1/drone_0_ego_planner_node/manager/max_acc)"
optimizer_acc="$(rosparam get /uav1/drone_0_ego_planner_node/optimization/max_acc)"
bspline_acc="$(rosparam get /uav1/drone_0_ego_planner_node/bspline/limit_acc)"

parameter_chain_valid=true
for value in "$manager_ceiling" "$optimizer_ceiling" "$bspline_ceiling" \
    "$dynamic_ceiling" "$filter_ceiling"; do
  if ! near "$value" 1.50; then parameter_chain_valid=false; fi
done
for value in "$manager_acc" "$optimizer_acc" "$bspline_acc"; do
  if ! near "$value" 0.50; then parameter_chain_valid=false; fi
done
if [[ "$policy_mode" != fixed ]] || ! near "$policy_value" "$fixed_v_max"; then
  parameter_chain_valid=false
fi

printf '{\n  "requested_v_max_mps": %s,\n  "filtered_v_max_mps": %s,\n  "applied_v_max_mps": %s,\n  "manager_ceiling_mps": %s,\n  "optimizer_ceiling_mps": %s,\n  "bspline_ceiling_mps": %s,\n  "dynamic_speed_limit_maximum_mps": %s,\n  "speed_safety_filter_maximum_mps": %s,\n  "policy_mode": "%s",\n  "fixed_speed_policy_mps": %s,\n  "manager_max_acc_mps2": %s,\n  "optimizer_max_acc_mps2": %s,\n  "bspline_max_acc_mps2": %s,\n  "topic_chain_valid": %s,\n  "parameter_chain_valid": %s\n}\n' \
  "${requested:-null}" "${filtered:-null}" "${applied:-null}" \
  "$manager_ceiling" "$optimizer_ceiling" "$bspline_ceiling" \
  "$dynamic_ceiling" "$filter_ceiling" "$policy_mode" "$policy_value" \
  "$manager_acc" "$optimizer_acc" "$bspline_acc" \
  "$chain_ready" "$parameter_chain_valid" >"$run_dir/speed_chain_preflight.json"

if ! "$chain_ready" || ! "$parameter_chain_valid"; then
  infrastructure_failure=true
  echo "speed_chain_preflight=FAIL" >>"$run_dir/run_metadata.txt"
  exit 20
fi
echo "startup_preflight=PASS speed_chain_preflight=PASS" >>"$run_dir/run_metadata.txt"

start_epoch="$(date +%s)"
while true; do
  if [[ -f "$run_dir/run_summary.json" ]]; then
    if python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); raise SystemExit(0 if d.get("mission",{}).get("done") or d.get("safety",{}).get("dangerous_terminal") else 1)' \
        "$run_dir/run_summary.json"; then
      echo "experiment_terminal=true" >>"$run_dir/run_metadata.txt"
      exit 0
    fi
  fi
  if [[ -z "$launch_pid" ]] || ! kill -0 "$launch_pid" 2>/dev/null; then
    if [[ -n "$launch_pid" ]]; then
      wait "$launch_pid" || true
      launch_pid=""
    fi
    sleep 2
    if [[ -f "$run_dir/run_summary.json" ]] && \
        python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); raise SystemExit(0 if d.get("mission",{}).get("done") or d.get("safety",{}).get("dangerous_terminal") else 1)' \
          "$run_dir/run_summary.json"; then
      echo "experiment_terminal=true" >>"$run_dir/run_metadata.txt"
      exit 0
    fi
    infrastructure_failure=true
    echo "launch_exited_without_experiment_terminal=true" >>"$run_dir/run_metadata.txt"
    exit 20
  fi
  now_epoch="$(date +%s)"
  if ((now_epoch - start_epoch >= wall_timeout)); then
    echo "wall_timeout=true" >>"$run_dir/run_metadata.txt"
    # A recorded planner/safety/mission failure is an experimental result and
    # must never be hidden by an automatic retry.
    if [[ -f "$run_dir/run_summary.json" ]] && \
        python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); p=d.get("planner",{}); s=d.get("safety",{}); m=d.get("mission",{}); raise SystemExit(0 if p.get("failure_episodes",0) or s.get("dangerous_terminal") or m.get("failure") else 1)' \
          "$run_dir/run_summary.json"; then
      echo "nonterminal_experimental_failure_preserved=true" >>"$run_dir/run_metadata.txt"
      exit 0
    fi
    infrastructure_failure=true
    exit 20
  fi
  sleep 5
done
