#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "$script_dir/../.." && pwd)"
exploration_root="$repo_root/runtime_artifacts/high_speed_exploration"
summary_script="$repo_root/AstraDrone_ros1_ws/src/learning_speed_rl/scripts/summarize_high_speed_exploration.py"
refinement_log="$exploration_root/boundary_refinement.log"

source /opt/ros/noetic/setup.bash
source "$repo_root/AstraDrone_ros1_ws/devel/setup.bash"

if [[ ! -f "$exploration_root/v200/run_summary.json" ]]; then
  echo "preserved first 2.0 m/s result is missing" >&2
  exit 2
fi
if [[ -e "$exploration_root/v200_retry" || -e "$exploration_root/v150" ]]; then
  echo "refusing to overwrite an existing retry/boundary run" >&2
  exit 2
fi

printf '%s refinement_start rule=2.0_retry_then_1.5_only_on_fail\n' \
  "$(date --iso-8601=seconds)" >>"$refinement_log"

retry_status=0
"$script_dir/high_speed_exploration.sh" --control --v-max 2.00 \
  --run-id v200_retry --wall-timeout 5400 || retry_status=$?
python3 "$summary_script" "$exploration_root"
if ((retry_status != 0)); then
  printf '%s refinement_stop run=v200_retry reason=orchestration_or_nonterminal status=%d\n' \
    "$(date --iso-8601=seconds)" "$retry_status" >>"$refinement_log"
  exit "$retry_status"
fi

retry_result="$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(next(r["result"] for r in d["runs"] if r["run_id"]=="v200_retry"))' \
  "$exploration_root/high_speed_summary.json")"
printf '%s retry_end result=%s\n' \
  "$(date --iso-8601=seconds)" "$retry_result" >>"$refinement_log"

if [[ "$retry_result" == PASS ]]; then
  printf '%s refinement_end v150=NOT_RUN reason=2.0_retry_pass\n' \
    "$(date --iso-8601=seconds)" >>"$refinement_log"
  exit 0
fi
if [[ "$retry_result" != FAIL ]]; then
  echo "v200_retry has no valid PASS/FAIL result" >&2
  exit 1
fi

printf '%s conditional_run_start id=v150 v_max=1.50\n' \
  "$(date --iso-8601=seconds)" >>"$refinement_log"
conditional_status=0
"$script_dir/high_speed_exploration.sh" --control --v-max 1.50 \
  --run-id v150 --wall-timeout 5400 || conditional_status=$?
python3 "$summary_script" "$exploration_root"
if ((conditional_status != 0)); then
  printf '%s refinement_stop run=v150 reason=orchestration_or_nonterminal status=%d\n' \
    "$(date --iso-8601=seconds)" "$conditional_status" >>"$refinement_log"
  exit "$conditional_status"
fi

conditional_result="$(python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); print(next(r["result"] for r in d["runs"] if r["run_id"]=="v150"))' \
  "$exploration_root/high_speed_summary.json")"
printf '%s refinement_end v150=%s\n' \
  "$(date --iso-8601=seconds)" "$conditional_result" >>"$refinement_log"
