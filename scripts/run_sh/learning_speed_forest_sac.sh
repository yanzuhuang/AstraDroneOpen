#!/usr/bin/env bash
# shellcheck disable=SC1090,SC1091
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
hector_overlay="${ASTRA_HECTOR_OVERLAY:-/tmp/astra_hector_training_overlay}"
hector_overlay_preparer="$repo_root/scripts/run_sh/prepare_hector_training_overlay.sh"
mode=""
run_id=""
gui=false
evaluation_seed=""
checkpoint_path=""
checkpoint_episode=""

while (($#)); do
  case "$1" in
    --mode) shift; mode="${1:-}" ;;
    --run-id) shift; run_id="${1:-}" ;;
    --gui) gui=true ;;
    --seed) shift; evaluation_seed="${1:-}" ;;
    --checkpoint) shift; checkpoint_path="${1:-}" ;;
    --checkpoint-episode) shift; checkpoint_episode="${1:-}" ;;
    --help)
      echo "learning_speed_forest_sac.sh --mode preflight|training|smoke|evaluation [options]"
      echo "  options: --run-id UNIQUE_RUN_ID --gui --seed 8|9 --checkpoint PATH --checkpoint-episode N --help"
      echo "  mode is required; GUI defaults to off; RUN_ID defaults to a mode-specific timestamped ID."
      echo "  preflight: starts Gazebo and a three-map reset lifecycle; no Replay, learner, or Episode execution; not a read-only check"
      echo "  training: starts 10000 completed training Episodes; existing qualification/approval requirements still apply"
      echo "  smoke: fixed 31 Episodes, 10 completed Episodes/map, three switches"
      echo "  evaluation: requires --seed 8|9 --checkpoint PATH --checkpoint-episode N"
      echo "  --seed, --checkpoint and --checkpoint-episode are used only by evaluation. No demo mode is supported."
      echo "  Non-help modes may prepare/build the Hector overlay. There is no --control, --rviz, --record or --stop option."
      echo "  Outputs: runtime_artifacts/rl_training/RUN_ID (training), runtime_artifacts/rl_evaluation/RUN_ID (evaluation),"
      echo "           runtime_artifacts/learning_speed/forest_randomization_training_integration_v2/RUN_ID (preflight/smoke)."
      echo "  Stop with Ctrl+C in the launching terminal; interruption is not a completed run. See docs/FINAL_RUNBOOK.md."
      exit 0
      ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

if [[ "$mode" != "preflight" && "$mode" != "training" && "$mode" != "smoke" && "$mode" != "evaluation" ]]; then
  echo "--mode preflight, training, smoke or evaluation is required" >&2
  exit 2
fi

"$hector_overlay_preparer" --overlay "$hector_overlay"

required_setup_files=(
  /opt/ros/noetic/setup.bash
  "$repo_root/simulation/sim_workspace/devel/setup.bash"
  "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"
  "$hector_overlay/devel/setup.bash"
)
for setup_file in "${required_setup_files[@]}"; do
  if [[ ! -f "$setup_file" ]]; then
    echo "required Forest SAC environment is missing: $setup_file" >&2
    exit 2
  fi
done
if [[ ! -d "$repo_root/runtime_artifacts/sac_python_packages" ]]; then
  echo "PyTorch dependency directory is missing" >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$hector_overlay/devel/setup.bash" --extend
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash" --extend
export PYTHONPATH="$repo_root/runtime_artifacts/sac_python_packages${PYTHONPATH:+:$PYTHONPATH}"
export ROS_MASTER_URI="${FOREST_ROS_MASTER_URI:-http://127.0.0.1:11331}"
export GAZEBO_MASTER_URI="${FOREST_GAZEBO_MASTER_URI:-http://127.0.0.1:11365}"

if rosnode list >/dev/null 2>&1; then
  echo "a ROS master is already reachable at $ROS_MASTER_URI" >&2
  exit 20
fi
residual_training="$(pgrep -af 'sac_training_runner.py|training_episode_reset_coordinator.py|forest_map_manager.py' || true)"
if [[ -n "$residual_training" ]]; then
  echo "residual Forest/training processes found:" >&2
  echo "$residual_training" >&2
  exit 20
fi

timestamp="$(date +%Y%m%d_%H%M%S)"
runner_mode="$mode"
total_training_episodes=10000
evaluation_episodes=100
smoke_test=false
episodes_per_map_block=100
case "$mode" in
  preflight)
    runner_mode=map_switch_preflight
    total_training_episodes=31
    episodes_per_map_block=10
    run_id="${run_id:-forest_map_switch_preflight_${timestamp}}"
    output_dir="$repo_root/runtime_artifacts/learning_speed/forest_randomization_training_integration_v2/$run_id"
    ;;
  training)
    run_id="${run_id:-forest_sac_training_10000ep_${timestamp}}"
    output_dir="$repo_root/runtime_artifacts/rl_training/$run_id"
    ;;
  smoke)
    runner_mode=training
    total_training_episodes=31
    smoke_test=true
    episodes_per_map_block=10
    run_id="${run_id:-forest_map_switch_smoke_31ep_${timestamp}}"
    output_dir="$repo_root/runtime_artifacts/learning_speed/forest_randomization_training_integration_v2/$run_id"
    ;;
  evaluation)
    if [[ "$evaluation_seed" != "8" && "$evaluation_seed" != "9" ]]; then
      echo "evaluation requires --seed 8 or --seed 9" >&2
      exit 2
    fi
    if [[ ! -f "$checkpoint_path" || ! "$checkpoint_episode" =~ ^[1-9][0-9]*$ ]]; then
      echo "evaluation requires an existing --checkpoint and positive --checkpoint-episode" >&2
      exit 2
    fi
    run_id="${run_id:-forest_eval_seed${evaluation_seed}_${timestamp}}"
    output_dir="$repo_root/runtime_artifacts/rl_evaluation/$run_id"
    ;;
esac

if ! mkdir -p "$(dirname "$output_dir")" || ! mkdir "$output_dir"; then
  echo "refusing to reuse output directory: $output_dir" >&2
  exit 2
fi
mkdir -p "$output_dir/logs/ros" "$output_dir/ros_home"
export ROS_HOME="$output_dir/ros_home"
export ROS_LOG_DIR="$output_dir/logs/ros"

echo "MODE=$mode"
echo "RUN_ID=$run_id"
echo "OUTPUT_DIR=$output_dir"
echo "ROS_MASTER_URI=$ROS_MASTER_URI"
echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"

launch_pid=""
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "$launch_pid" ]] && kill -0 "$launch_pid" 2>/dev/null; then
    kill -INT "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

roslaunch hector_ego_training_backend hector_forest_sac_training.launch \
  "output_dir:=$output_dir" \
  "gui:=$gui" \
  "runner_mode:=$runner_mode" \
  "total_training_episodes:=$total_training_episodes" \
  "evaluation_episodes:=$evaluation_episodes" \
  "evaluation_forest_seed:=${evaluation_seed:-8}" \
  "evaluation_checkpoint_path:=$checkpoint_path" \
  "evaluation_checkpoint_episode:=${checkpoint_episode:-0}" \
  "smoke_test:=$smoke_test" \
  "episodes_per_map_block:=$episodes_per_map_block" \
  "run_id:=$run_id" \
  >"$output_dir/logs/console.log" 2>&1 &
launch_pid=$!
status=0
wait "$launch_pid" || status=$?
launch_pid=""
trap - EXIT INT TERM

summary_path="$output_dir/sac_runtime_summary.json"
if [[ "$mode" == "preflight" ]]; then
  summary_path="$output_dir/qualification_summary.json"
fi
if ((status == 0)); then
  if [[ ! -f "$summary_path" ]]; then
    echo "runner summary is missing; treating roslaunch cleanup status as failure" >&2
    status=2
  elif ! python3 -c 'import json,sys; p=json.load(open(sys.argv[1], encoding="utf-8")); ok=(p.get("verdict")=="FOREST MAP-SWITCH PREFLIGHT PASS") if sys.argv[2]=="preflight" else (p.get("status")=="completed"); raise SystemExit(0 if ok else 2)' "$summary_path" "$mode"; then
    echo "runtime summary is not completed; preserving NO-GO despite roslaunch cleanup status 0" >&2
    status=2
  fi
fi

echo "roslaunch exit status: $status"
echo "summary: $summary_path"
if ((status != 0)); then
  tail -n 100 "$output_dir/logs/console.log" >&2 || true
fi
exit "$status"
