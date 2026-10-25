#!/usr/bin/env bash
# Each attempt uses a newly launched copy of the unchanged TESSE physics scene.
set -eo pipefail
cd "$(dirname "$0")"
source env.sh
export LAB4_SIM_HEADLESS="${LAB4_SIM_HEADLESS:-0}"
export LAB4_EXIT_OBSERVATION="${LAB4_EXIT_OBSERVATION:-8}"
mkdir -p results
exec 9> results/.sub7_race.lock
flock -n 9 || { echo 'Another racing session is active.' >&2; exit 2; }
batch="verify_3d_$(date +%Y%m%d_%H%M%S_%N)"
export LAB4_PLANNER_PROFILE="$PWD/optimization/planner_profiles/sub7.yaml"
total=$(python3 -c 'import sys,yaml; print(sum(yaml.safe_load(open(sys.argv[1]))["segment_durations"]))' "$LAB4_PLANNER_PROFILE")
for index in 1 2 3; do
  prefix="${batch}_${index}"
  mkdir -p "results/${prefix}_startup"
  python3 -u scripts/prepare_simulator.py "$PWD/results/${prefix}_startup"
  bash optimization/run_trial.sh "$PWD/optimization/profiles/sub7.yaml" "$total" "$prefix" 2.67 75 > "results/${prefix}_dispatch.log" 2>&1 || true
  python3 optimization/collect_results.py
  printf 'Cold-start qualification %s complete
' "$index"
done
python3 - "$batch" <<'PY'
import json,sys
from pathlib import Path
batch=sys.argv[1]
passed=0
for index in (1,2,3):
    path=Path('results')/(batch+'_'+str(index)+'_01')/'race_acceptance.json'
    if not path.exists():
        print('Trial %d: missing acceptance result'%index)
        continue
    r=json.loads(path.read_text())
    ok=r['passed'] and r['first_gate_to_last_gate_ms']<7000
    passed+=int(ok)
    print('Trial %d: %s, gates %d/12, first-to-last %s ms' %
          (index,'PASS <7s' if ok else 'FAIL',r['passed_gates'],r['first_gate_to_last_gate_ms']))
print('Independent fresh starts under 7s: %d/3'%passed)
raise SystemExit(0 if passed==3 else 1)
PY
