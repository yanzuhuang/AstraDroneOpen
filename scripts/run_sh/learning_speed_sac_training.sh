#!/usr/bin/env bash
# ROS/catkin setup paths are validated for existence before these dynamic sources.
# shellcheck disable=SC1090,SC1091
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
hector_overlay="${ASTRA_HECTOR_OVERLAY:-/tmp/astra_hector_training_overlay}"
hector_overlay_preparer="$repo_root/scripts/run_sh/prepare_hector_training_overlay.sh"
training_root="$repo_root/runtime_artifacts/rl_training"
sac_config="$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/config/sac_training_v1.yaml"

gui=false
run_id=""
while (($#)); do
  case "$1" in
    --gui)
      gui=true
      ;;
    --run-id)
      shift
      run_id="${1:-}"
      ;;
    --help)
      echo "learning_speed_sac_training.sh [--gui] [--run-id sac_training_10000ep_YYYYMMDD_HHMMSS] [--help]"
      echo "Starts the formal 10000-Episode SAC training run and follows key logs."
      echo "Defaults: GUI off; RUN_ID generated from the current timestamp; invoking without options starts training."
      echo "Existing qualification/approval requirements still apply; this entry does not grant training approval."
      echo "May prepare/build the Hector overlay. No --mode, --control, --rviz, --record, --stop or resume option."
      echo "Output: runtime_artifacts/rl_training/RUN_ID/; existing directories are rejected."
      echo "Stop with Ctrl+C in the launching terminal; this records an intentional interruption, not training completion."
      echo "Read sac_runtime_summary.json; process exit code alone does not establish training success. See docs/FINAL_RUNBOOK.md."
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

if [[ -z "$run_id" ]]; then
  run_id="sac_training_10000ep_$(date +%Y%m%d_%H%M%S)"
fi
if [[ ! "$run_id" =~ ^sac_training_10000ep_[0-9]{8}_[0-9]{6}$ ]]; then
  echo "RUN_ID must match sac_training_10000ep_YYYYMMDD_HHMMSS" >&2
  exit 2
fi

"$hector_overlay_preparer" --overlay "$hector_overlay"

required_setup_files=(
  "/opt/ros/noetic/setup.bash"
  "$repo_root/simulation/sim_workspace/devel/setup.bash"
  "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
  "$hector_overlay/devel/setup.bash"
)
for setup_file in "${required_setup_files[@]}"; do
  if [[ ! -f "$setup_file" ]]; then
    echo "required training environment is missing: $setup_file" >&2
    echo "after a reboot, run studynote.md step 1 before starting training" >&2
    exit 2
  fi
done
if [[ ! -f "$sac_config" ]]; then
  echo "formal SAC config is missing: $sac_config" >&2
  exit 2
fi
if [[ ! -d "$repo_root/runtime_artifacts/sac_python_packages" ]]; then
  echo "PyTorch dependency directory is missing: runtime_artifacts/sac_python_packages" >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
source "$hector_overlay/devel/setup.bash" --extend
export PYTHONPATH="$repo_root/runtime_artifacts/sac_python_packages${PYTHONPATH:+:$PYTHONPATH}"

if rosnode list >/dev/null 2>&1; then
  echo "an existing ROS master is reachable; refusing to mix training stacks" >&2
  exit 20
fi
residual_processes="$(pgrep -af '(^|/)(gzserver|gzclient)( |$)|sac_training_runner.py|training_episode_reset_coordinator.py' || true)"
if [[ -n "$residual_processes" ]]; then
  echo "residual Gazebo/training processes were found; refusing to start:" >&2
  echo "$residual_processes" >&2
  exit 20
fi

run_dir="$training_root/$run_id"
mkdir -p "$training_root"
if ! mkdir "$run_dir"; then
  echo "refusing to reuse or overwrite an existing training RUN_ID: $run_dir" >&2
  exit 2
fi
mkdir -p "$run_dir/logs/ros" "$run_dir/ros_home"
export ROS_HOME="$run_dir/ros_home"
export ROS_LOG_DIR="$run_dir/logs/ros"

echo "RUN_ID=$run_id"
echo "SAC_OUTPUT=$run_dir"
echo "FULL_CONSOLE_LOG=$run_dir/logs/training_console.log"
echo "KEY_LOG_MONITOR=automatic"
echo "Press Ctrl+C only to record an intentional interrupted/NO-GO run."

launch_pid=""
monitor_pid=""

cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$monitor_pid" ]] && kill -0 "$monitor_pid" 2>/dev/null; then
    kill "$monitor_pid" 2>/dev/null || true
    wait "$monitor_pid" 2>/dev/null || true
  fi
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

follow_key_logs() {
  local runner_log=""
  local coordinator_log=""
  while kill -0 "$launch_pid" 2>/dev/null; do
    runner_log="$(find "$run_dir/logs/ros" -type f \
      -name 'uav1-sac_training_runner-*.log' -print -quit 2>/dev/null || true)"
    coordinator_log="$(find "$run_dir/logs/ros" -type f \
      -name 'uav1-training_episode_reset_coordinator-*.log' -print -quit 2>/dev/null || true)"
    if [[ -n "$runner_log" && -n "$coordinator_log" ]]; then
      break
    fi
    sleep 1
  done
  if [[ -z "$runner_log" || -z "$coordinator_log" ]]; then
    return 0
  fi

  echo "[SAC LOG MONITOR] following runner and Episode/reset progress"
  tail --pid="$launch_pid" -n 100 -F "$runner_log" "$coordinator_log" 2>/dev/null \
    | grep --line-buffered -E \
      '\[SAC TRAINING\]|stochastic training enabled|checkpoint|Episode count|SAC training failed|\[TRAINING EPISODE\] (EPISODE_|RESET_|SAC_WARMUP|QUALIFICATION_)'
}

roslaunch hector_ego_training_backend hector_worksite_sac_training.launch \
  "output_dir:=$run_dir" \
  "gui:=$gui" \
  runner_mode:=training \
  total_training_episodes:=10000 \
  max_episode_time:=55.0 \
  "run_id:=$run_id" \
  "sac_config:=$sac_config" \
  > "$run_dir/logs/training_console.log" 2>&1 &
launch_pid=$!

follow_key_logs &
monitor_pid=$!

launch_status=0
wait "$launch_pid" || launch_status=$?
wait "$monitor_pid" 2>/dev/null || true
launch_pid=""
monitor_pid=""
trap - EXIT INT TERM

echo "training roslaunch exit status: $launch_status"
echo "summary: $run_dir/sac_runtime_summary.json"
if ((launch_status != 0)); then
  echo "roslaunch failed; last 80 lines of the full console follow:" >&2
  tail -n 80 "$run_dir/logs/training_console.log" >&2 || true
fi
exit "$launch_status"
