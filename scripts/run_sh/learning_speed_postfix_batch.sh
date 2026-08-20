#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
calibration_root="$repo_root/runtime_artifacts/learning_speed/calibration"
postfix_root="$calibration_root/postfix_stage1_20260818"
audit_script="$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/summarize_postfix_calibration.py"
speeds=(0.30 0.50 0.75 1.00 1.25 1.50 1.75)

mkdir -p "$postfix_root"
for environment in A B; do
  for speed in "${speeds[@]}"; do
    code="$(awk -v value="$speed" 'BEGIN { printf "%03d", value * 100 }')"
    run_id="postfix_${environment}_v${code}_r01"
    attempt=1
    if [[ -e "$postfix_root/$run_id" ]]; then
      attempt=2
    fi
    "$script_dir/learning_speed_manual_run.sh" --control \
      --environment "$environment" --v-max "$speed" --run-id "$run_id" --attempt "$attempt" \
      --speed-ceiling 2.00 --calibration-root "$postfix_root" --calibration-generation post_fix
    python3 "$audit_script" --postfix-root "$postfix_root" --output-root "$calibration_root" --audit-run "$run_id"
  done
done
python3 "$audit_script" --postfix-root "$postfix_root" --output-root "$calibration_root"
