#!/usr/bin/env bash
set -eo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$BASE"
source env.sh
export LAB4_SIM_HEADLESS=0 LAB4_EXIT_OBSERVATION=8
unset LAB4_KEEP_FINAL_HOLD
manifest="$(realpath "$1")"
mkdir -p results
exec 9>results/.sub7_race.lock
flock -n 9 || { echo 'A flight is already running'; exit 2; }
batch="search_$(date +%Y%m%d_%H%M%S_%N)"
while IFS=$'\t' read -r label planner controller; do
  controller="${controller%$'\r'}"
  prefix="${batch}_${label}"
  export LAB4_PLANNER_PROFILE="$BASE/$planner"
  duration=$(python3 -c 'import sys,yaml; print(sum(yaml.safe_load(open(sys.argv[1]))["segment_durations"]))' "$LAB4_PLANNER_PROFILE")
  mkdir -p "results/${prefix}_startup"
  echo "START $prefix"
  if ! python3 -u scripts/prepare_simulator.py "$PWD/results/${prefix}_startup" >"results/${prefix}_startup/dispatch.log" 2>&1 9>&-; then
    echo 'STARTUP FAILED'; continue
  fi
  status=0
  bash optimization/run_trial.sh "$BASE/$controller" "$duration" "$prefix" 2.67 75 >"results/${prefix}_dispatch.log" 2>&1 9>&- || status=$?
  tail -5 "results/${prefix}_dispatch.log"
  directory="results/${prefix}_01"
  if [[ -f "$directory/race_acceptance.json" ]]; then
    python3 scripts/audit_race_numerics.py "$directory" >"results/${prefix}_numerics.log" 2>&1 || true
    python3 optimization/inspect_all_crossings.py "$directory" >"results/${prefix}_all_gates.log" 2>&1 || true
    python3 - "$directory" <<'SUMMARY'
import json,sys
from pathlib import Path
p=Path(sys.argv[1]);r=json.loads((p/'race_acceptance.json').read_text())
print(json.dumps(dict(run=p.name,passed=r['passed'],gates=r['passed_gates'],actual_ms=r['first_gate_to_last_gate_ms'],checks=r['checks'])))
f=p/'analysis/all_gate_diagnostics.json'
if f.exists(): print('DIAGNOSTIC YZ',[(a['gate'],round(a.get('local_y_m',999),2),round(a.get('local_z_m',999),2)) for a in json.loads(f.read_text())['crossings']])
SUMMARY
  else
    echo "Incomplete evidence; dispatch exit code $status"
  fi
done < "$manifest"
