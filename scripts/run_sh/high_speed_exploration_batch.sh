#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
exploration_root="$repo_root/runtime_artifacts/high_speed_exploration"
summary_script="$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/summarize_high_speed_exploration.py"
mkdir -p "$exploration_root"

source /opt/ros/noetic/setup.bash
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"

runs=("0.50:v050" "1.00:v100" "2.00:v200")
printf '%s exploration_start strict_early_stop=true\n' \
  "$(date --iso-8601=seconds)" >>"$exploration_root/batch.log"

for specification in "${runs[@]}"; do
  speed="${specification%%:*}"
  run_id="${specification#*:}"
  printf '%s run_start id=%s v_max=%s\n' \
    "$(date --iso-8601=seconds)" "$run_id" "$speed" >>"$exploration_root/batch.log"

  run_status=0
  "$script_dir/high_speed_exploration.sh" --control --v-max "$speed" \
    --run-id "$run_id" --wall-timeout 5400 || run_status=$?
  python3 "$summary_script" "$exploration_root"

  if ((run_status != 0)); then
    printf '%s early_stop id=%s reason=orchestration_or_nonterminal_failure status=%d\n' \
      "$(date --iso-8601=seconds)" "$run_id" "$run_status" >>"$exploration_root/batch.log"
    exit "$run_status"
  fi

  if ! python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); r=next(x for x in d["runs"] if x["run_id"]==sys.argv[2]); raise SystemExit(0 if r["result"]=="PASS" else 1)' \
      "$exploration_root/high_speed_summary.json" "$run_id"; then
    printf '%s early_stop id=%s reason=formal_run_fail\n' \
      "$(date --iso-8601=seconds)" "$run_id" >>"$exploration_root/batch.log"
    exit 1
  fi

  printf '%s run_pass id=%s\n' \
    "$(date --iso-8601=seconds)" "$run_id" >>"$exploration_root/batch.log"
done

python3 "$summary_script" "$exploration_root"
printf '%s exploration_end all_levels_passed=true\n' \
  "$(date --iso-8601=seconds)" >>"$exploration_root/batch.log"
