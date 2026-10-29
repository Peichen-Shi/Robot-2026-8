#!/usr/bin/env bash
# Three fresh visible Unity simulations; record at least sixty stable hold seconds.
set -eo pipefail
cd "${1:-$(dirname -- "${BASH_SOURCE[0]}")}"
source env.sh
export LAB4_SIM_HEADLESS=0 LAB4_EXIT_OBSERVATION=65
unset LAB4_KEEP_FINAL_HOLD
export LAB4_PLANNER_PROFILE="$PWD/optimization/planner_profiles/sub4.yaml"
mkdir -p results
exec 9> results/.sub7_race.lock
flock -n 9 || { echo 'Another flight is active'; exit 2; }
batch="qualify_sub4_$(date +%Y%m%d_%H%M%S_%N)"
duration=$(python3 -c 'import yaml,sys; print(sum(yaml.safe_load(open(sys.argv[1]))["segment_durations"]))' "$LAB4_PLANNER_PROFILE")
for index in 1 2 3; do
  python3 - <<'FROZEN'
import pathlib,json,hashlib
r=pathlib.Path.cwd();m=json.loads((r/'results/sub4_frozen_manifest.json').read_text())
for path,digest in m.items():
 p=pathlib.Path.home()/'vnav-builds/lab4/lab4.x86_64' if path=='simulator_binary' else r/path
 if hashlib.sha256(p.read_bytes()).hexdigest()!=digest:raise SystemExit('Frozen qualification input changed: '+path)
FROZEN
  prefix="${batch}_${index}"
  echo "START qualification $index/3: $prefix; 65 second terminal observation"
  mkdir -p "results/${prefix}_startup"
  python3 -u scripts/prepare_simulator.py "$PWD/results/${prefix}_startup" >"results/${prefix}_startup/dispatch.log" 2>&1 9>&-
  status=0
  bash optimization/run_trial.sh "$PWD/optimization/profiles/sub4.yaml" "$duration" "$prefix" 2.67 75 >"results/${prefix}_dispatch.log" 2>&1 9>&- || status=$?
  tail -5 "results/${prefix}_dispatch.log"
  if [[ "$status" == 0 ]]; then
    python3 scripts/audit_race_numerics.py "results/${prefix}_01" >"results/${prefix}_numeric.log" 2>&1 || true
    python3 scripts/check_sub4_qualification.py "results/${prefix}_01" || true
  fi
done
python3 - "$batch" <<'SUMMARY'
import json,sys,pathlib
r=pathlib.Path('results');batch=sys.argv[1];attempts=[]
for index in (1,2,3):
 p=r/(batch+'_'+str(index)+'_01')/'sub4_qualification.json'
 attempts.append(json.loads(p.read_text()) if p.exists() else {'run_id':p.parent.name,'qualified':False,'reason':'missing complete evidence'})
summary={'batch':batch,'rule':'3/3 fresh visible simulations: actual 12-gate race <4s, no collisions/faults, independent reference audit, continuous stable terminal hold >=60s','attempts':attempts,'qualified':all(a['qualified'] for a in attempts)}
(r/(batch+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
raise SystemExit(0 if summary['qualified'] else 1)
SUMMARY
