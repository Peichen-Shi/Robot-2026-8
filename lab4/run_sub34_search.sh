#!/usr/bin/env bash
# Explore reference candidates; every output is unqualified until flown.
set -eo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$BASE"
source env.sh
export LAB4_SIM_HEADLESS=0 LAB4_EXIT_OBSERVATION=8
unset LAB4_KEEP_FINAL_HOLD
candidate="${1:-$BASE/optimization/candidates/sub34_screen_v2_20261031}"
candidate="$(realpath "$candidate")"
mkdir -p results
exec 9>results/.sub7_race.lock
flock -n 9 || { echo 'A flight is already running; stop it before starting a search'; exit 2; }
batch="sub34_search_$(date +%Y%m%d_%H%M%S_%N)"
for yaw in 0.2 0.1 0.0; do
  prefix="${batch}_yaw${yaw/./p}"
  export LAB4_PLANNER_PROFILE="$candidate/screened_planner.yaml"
  controller="$candidate/controller_yaw${yaw}.yaml"
  duration=$(python3 -c 'import sys,yaml; print(sum(yaml.safe_load(open(sys.argv[1]))["segment_durations"]))' "$LAB4_PLANNER_PROFILE")
  mkdir -p "results/${prefix}_startup"
  echo "START $prefix; 3.38 s is a reference interval, not a measured result"
  if ! python3 -u scripts/prepare_simulator.py "$PWD/results/${prefix}_startup" >"results/${prefix}_startup/dispatch.log" 2>&1 9>&-; then
    echo "Startup failed; preserve logs: results/${prefix}_startup"; continue
  fi
  status=0
  bash optimization/run_trial.sh "$controller" "$duration" "$prefix" 2.67 75 >"results/${prefix}_dispatch.log" 2>&1 9>&- || status=$?
  tail -5 "results/${prefix}_dispatch.log"
  if [[ -f "results/${prefix}_01/race_acceptance.json" ]]; then
    python3 scripts/audit_race_numerics.py "results/${prefix}_01" >"results/${prefix}_numerics.log" 2>&1 || true
    python3 - "$prefix" "$status" <<'SUMMARY'
import json,sys,pathlib
p=pathlib.Path('results')/(sys.argv[1]+'_01')/'race_acceptance.json'
r=json.loads(p.read_text())
print(json.dumps(dict(run=p.parent.name,passed=r['passed'],gates=r['passed_gates'],actual_ms=r['first_gate_to_last_gate_ms'],checks=r['checks'],note='Exploration only; still requires three independent long-hold qualifications')))
SUMMARY
  else
    echo "No complete race evidence; dispatch status $status"
  fi
done
