#!/usr/bin/env bash
# Recovered setup chain: 2c30cf3360f3d5c026f41da4c29257139ff9abb3:
# scripts/run_sh/learning_speed_sac_training.sh (deleted by fa186ea8bbbf6fd6aa3d2ae0bff1e0cf65f6aa75).
# Adapted to evaluation only, current script location and paired Worksite/Forest batch.
set -eo pipefail
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo 'Usage: learning_speed_worksite_evaluation.sh --prepare|--run [--episodes N] --output runtime_artifacts/NAME_TIMESTAMP'
  echo 'prepare is offline; run executes the frozen paired evaluation Episodes, never training.'
  exit 0
fi
if [[ "${1:-}" == --run ]]; then
  source /opt/ros/noetic/setup.bash
  source "$repo_root/simulation/sim_workspace/devel/setup.bash"
  source "${ASTRA_HECTOR_OVERLAY:-/tmp/astra_hector_training_overlay}/devel/setup.bash" --extend
  source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash" --extend
  export PYTHONPATH="$repo_root/runtime_artifacts/sac_python_packages${PYTHONPATH:+:$PYTHONPATH}"
fi
exec python3 "$repo_root/scripts/tool/sac_formal_evaluation.py" "$@"
