#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
qualification_root="${1:-$repo_root/runtime_artifacts/learning_speed/high_speed_progressive_qualification_20260820}"
analyzer="$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/summarize_high_speed_progressive_qualification.py"
baseline="$repo_root/runtime_artifacts/learning_speed/calibration/stage1_reward_calibration_postfix_analysis.json"
report="$repo_root/high_speed_progressive_qualification_report.md"
speeds=(1.75 2.00 2.50 3.00 3.50)

if [[ -e "$qualification_root" ]]; then
  echo "refusing to overwrite qualification root: $qualification_root" >&2
  exit 2
fi
mkdir -p "$qualification_root"

source /opt/ros/noetic/setup.bash
source "$repo_root/simulation/sim_workspace/devel/setup.bash"
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"

stop_reason="all_requested_speeds_completed"
for speed in "${speeds[@]}"; do
  code="$(awk -v value="$speed" 'BEGIN {printf "%03d", value * 100}')"
  run_id="hsq_B_v${code}_r01"
  run_dir="$qualification_root/$run_id"
  set +e
  "$script_dir/learning_speed_manual_run.sh" --control \
    --environment B --v-max "$speed" --run-id "$run_id" --attempt 1 \
    --speed-ceiling 4.00 --max-acc 3.00 \
    --calibration-root "$qualification_root" \
    --calibration-generation high_speed_qualification \
    --record-control-chain
  runner_status=$?
  set -e

  if [[ -f "$run_dir/run_summary.json" && -f "$run_dir/control_chain.bag" ]]; then
    python3 "$analyzer" --run-dir "$run_dir" \
      >"$run_dir/analyzer_stdout.log" 2>&1 || {
        stop_reason="analysis_failed_${run_id}"
        break
      }
  fi
  if ((runner_status != 0)); then
    stop_reason="runner_status_${runner_status}_${run_id}"
    break
  fi
  if [[ ! -f "$run_dir/qualification_run_analysis.json" ]]; then
    stop_reason="missing_analysis_${run_id}"
    break
  fi
  if ! python3 -c 'import json,sys; data=json.load(open(sys.argv[1])); raise SystemExit(0 if data.get("pass") else 1)' \
      "$run_dir/qualification_run_analysis.json"; then
    stop_reason="formal_failure_${run_id}"
    break
  fi
done

python3 "$analyzer" --qualification-root "$qualification_root" \
  --baseline-analysis "$baseline" --report "$report" \
  >"$qualification_root/aggregate_stdout.log" 2>&1

printf '{\n  "schema_version": "high_speed_progressive_batch_v1.0",\n  "stop_reason": "%s",\n  "automatic_retry": false,\n  "environment": "B",\n  "requested_speeds_mps": [1.75, 2.0, 2.5, 3.0, 3.5]\n}\n' \
  "$stop_reason" >"$qualification_root/batch_state.json"

if [[ "$stop_reason" != all_requested_speeds_completed ]]; then
  exit 1
fi
