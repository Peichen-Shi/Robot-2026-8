#!/usr/bin/env bash
set -eo pipefail
cd "${1:-$HOME/lab4_sub4_20261029}"
source env.sh
export LAB4_SIM_HEADLESS=0 LAB4_EXIT_OBSERVATION=8
unset LAB4_KEEP_FINAL_HOLD
mkdir -p results
exec 9>results/.sub7_race.lock
flock -n 9 || { echo 'An existing race is active'; exit 2; }
for specification in '4.4 balanced' '3.9 balanced' '3.9 balanced_bias'; do
  read -r target controller <<< "$specification"
  prefix="balanced_${target/./p}_${controller}"
  if [[ -d "results/${prefix}_01" ]]; then
    echo "Preserving existing trial $prefix"
    continue
  fi
  export LAB4_PLANNER_PROFILE="$PWD/optimization/phase2/planner_${controller}_${target}.yaml"
  controller=identified
  duration=$(python3 -c 'import sys,yaml; print(sum(yaml.safe_load(open(sys.argv[1]))["segment_durations"]))' "$LAB4_PLANNER_PROFILE")
  mkdir -p "results/${prefix}_startup"
  echo "START $prefix"
  if ! python3 -u scripts/prepare_simulator.py "$PWD/results/${prefix}_startup" >"results/${prefix}_startup/dispatch.log" 2>&1 9>&-; then
    echo "STARTUP FAILED $prefix"; continue
  fi
  status=0
  bash optimization/run_trial.sh "$PWD/optimization/phase2/controller_${controller}.yaml" "$duration" "$prefix" 2.67 75 >"results/${prefix}_dispatch.log" 2>&1 9>&- || status=$?
  tail -6 "results/${prefix}_dispatch.log"
  python3 - "$prefix" "$status" <<'PY'
import json,sys
from pathlib import Path
prefix=sys.argv[1];p=Path('results')/(prefix+'_01')/'race_acceptance.json'
if p.exists():
 r=json.loads(p.read_text());print(json.dumps(dict(run=prefix,passed=r['passed'],gates=r['passed_gates'],milliseconds=r['first_gate_to_last_gate_ms'],checks=r['checks'])),flush=True)
else:print('No completed acceptance evidence: '+prefix,flush=True)
PY
done
python3 optimization/collect_results.py
