#!/usr/bin/env bash
set -u

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
batch_log="$repo_root/runtime_artifacts/fixed_speed_baseline/batch.log"
mkdir -p "$(dirname "$batch_log")"

# v020_r01 is deliberately omitted: it is the preserved nonterminal sample
# from an orchestration-session timeout, not a valid baseline trial.
runs=(
  "0.20:v020_r02" "0.20:v020_r03" "0.20:v020_r04"
  "0.16:v016_r01" "0.16:v016_r02" "0.16:v016_r03"
  "0.12:v012_r01" "0.12:v012_r02" "0.12:v012_r03"
  "0.08:v008_r01" "0.08:v008_r02" "0.08:v008_r03"
)

printf '%s batch_start\n' "$(date --iso-8601=seconds)" >>"$batch_log"
for specification in "${runs[@]}"; do
  speed="${specification%%:*}"
  run_id="${specification#*:}"
  printf '%s run_start id=%s v_max=%s\n' \
    "$(date --iso-8601=seconds)" "$run_id" "$speed" >>"$batch_log"
  if "$script_dir/fixed_speed_baseline.sh" --control --v-max "$speed" \
      --run-id "$run_id" --wall-timeout 5400; then
    result=terminal
  elif [[ -f "$repo_root/runtime_artifacts/fixed_speed_baseline/runs/$run_id/run_summary.json" ]] &&
      rg -q '"done"[[:space:]]*:[[:space:]]*true' \
        "$repo_root/runtime_artifacts/fixed_speed_baseline/runs/$run_id/run_summary.json"; then
    result=terminal_reconciled_from_run_summary
  else
    result=nonterminal_or_launch_error
  fi
  printf '%s run_end id=%s result=%s\n' \
    "$(date --iso-8601=seconds)" "$run_id" "$result" >>"$batch_log"
  sleep 5
done
printf '%s batch_end\n' "$(date --iso-8601=seconds)" >>"$batch_log"
