#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../../.." && pwd)"

namespace="uav1"
run_id=""
start_observation_c=true
while (($#)); do
  case "$1" in
    --namespace)
      shift
      namespace="${1:-}"
      ;;
    --run-id)
      shift
      run_id="${1:-}"
      ;;
    --observation-c-running)
      start_observation_c=false
      ;;
    --help)
      echo "learning_speed_calibration.sh [--namespace uav1] [--run-id ID] [--observation-c-running]"
      echo "Attaches a read-only recorder to an already running Learning Speed stack."
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
  shift
done

if [[ ! "$namespace" =~ ^[a-zA-Z0-9][a-zA-Z0-9_-]*$ ]]; then
  echo "--namespace must contain only letters, digits, underscore or dash" >&2
  exit 2
fi
if [[ -z "$run_id" ]]; then
  run_id="${namespace}_$(date +%Y%m%d_%H%M%S)"
fi
if [[ ! "$run_id" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]]; then
  echo "--run-id must contain only letters, digits, dot, underscore or dash" >&2
  exit 2
fi

run_dir="$repo_root/runtime_artifacts/learning_speed/calibration/$run_id"
if [[ -e "$run_dir" ]]; then
  echo "refusing to overwrite existing calibration run: $run_dir" >&2
  exit 2
fi

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"

required_topics=(
  "/$namespace/Odometry"
  "/$namespace/stage3/cloud_registered_filtered"
  "/$namespace/planning/bspline"
  "/$namespace/learning_speed/raw_v_max"
  "/$namespace/learning_speed/v_max"
  "/$namespace/learning_speed/action_stamped"
  "/$namespace/learning_speed/applied_v_max"
)
for topic in "${required_topics[@]}"; do
  if ! rostopic info "$topic" >/dev/null 2>&1; then
    echo "required topic is unavailable: $topic" >&2
    echo "start the stack with explicit Learning Speed enabled before attaching the recorder" >&2
    exit 1
  fi
done

echo "calibration output: $run_dir"
echo "collector is read-only; reward and SAC training remain disabled"
exec roslaunch learning_speed_rl calibration_data_collection.launch \
  "namespace:=$namespace" \
  "run_id:=$run_id" \
  "output_dir:=$run_dir" \
  "start_observation_c:=$start_observation_c"
