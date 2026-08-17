#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
calibration_root="$repo_root/runtime_artifacts/learning_speed/calibration"
summary_script="$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/summarize_manual_calibration.py"
speeds=(0.30 0.50 0.75 1.00 1.25 1.50)

mkdir -p "$calibration_root"
printf '%s manual_calibration_start uav=UAV1 attempts_per_condition=1 infrastructure_retry_limit=1\n' \
  "$(date --iso-8601=seconds)" >>"$calibration_root/manual_batch.log"

for environment in A B; do
  for speed in "${speeds[@]}"; do
    speed_code="$(awk -v value="$speed" 'BEGIN {printf "%03d", value * 100}')"
    logical_run_id="${environment}_v${speed_code}_r01"
    attempt=1
    first_dir="$calibration_root/$logical_run_id"
    retry_dir="$calibration_root/${logical_run_id}_infra_retry"
    if [[ -f "$first_dir/run_summary.json" ]] && \
        python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); raise SystemExit(0 if d.get("mission",{}).get("done") or d.get("safety",{}).get("dangerous_terminal") else 1)' \
          "$first_dir/run_summary.json"; then
      printf '%s run_skip_existing_terminal logical_id=%s attempt=1\n' \
        "$(date --iso-8601=seconds)" "$logical_run_id" \
        >>"$calibration_root/manual_batch.log"
      continue
    elif [[ -e "$first_dir" ]]; then
      attempt=2
    fi
    if [[ -f "$retry_dir/run_summary.json" ]] && \
        python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); raise SystemExit(0 if d.get("mission",{}).get("done") or d.get("safety",{}).get("dangerous_terminal") else 1)' \
          "$retry_dir/run_summary.json"; then
      printf '%s run_skip_existing_terminal logical_id=%s attempt=2\n' \
        "$(date --iso-8601=seconds)" "$logical_run_id" \
        >>"$calibration_root/manual_batch.log"
      continue
    elif [[ -e "$retry_dir" ]]; then
      printf '%s batch_stop logical_id=%s reason=both_attempt_directories_exist_without_terminal\n' \
        "$(date --iso-8601=seconds)" "$logical_run_id" \
        >>"$calibration_root/manual_batch.log"
      exit 20
    fi
    printf '%s run_start logical_id=%s environment=%s fixed_v_max=%s attempt=%s\n' \
      "$(date --iso-8601=seconds)" "$logical_run_id" "$environment" "$speed" \
      "$attempt" \
      >>"$calibration_root/manual_batch.log"
    status=0
    "$script_dir/learning_speed_manual_run.sh" --control \
      --environment "$environment" --v-max "$speed" \
      --run-id "$logical_run_id" --attempt "$attempt" || status=$?

    if ((status == 20 && attempt == 1)); then
      printf '%s infrastructure_retry logical_id=%s attempt=2\n' \
        "$(date --iso-8601=seconds)" "$logical_run_id" \
        >>"$calibration_root/manual_batch.log"
      sleep 8
      status=0
      "$script_dir/learning_speed_manual_run.sh" --control \
        --environment "$environment" --v-max "$speed" \
        --run-id "$logical_run_id" --attempt 2 || status=$?
    fi
    if ((status != 0)); then
      printf '%s batch_stop logical_id=%s status=%s reason=orchestration_failure\n' \
        "$(date --iso-8601=seconds)" "$logical_run_id" "$status" \
        >>"$calibration_root/manual_batch.log"
      python3 "$summary_script" "$calibration_root"
      exit "$status"
    fi
    python3 "$summary_script" "$calibration_root" >/dev/null
    printf '%s run_preserved logical_id=%s\n' \
      "$(date --iso-8601=seconds)" "$logical_run_id" \
      >>"$calibration_root/manual_batch.log"
    sleep 8
  done
done

python3 "$summary_script" "$calibration_root"
printf '%s manual_calibration_end logical_runs=12\n' \
  "$(date --iso-8601=seconds)" >>"$calibration_root/manual_batch.log"
