#!/usr/bin/env bash
# Three independent visible flights; keep all original flight/hold checks.
set -eo pipefail
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$BASE"
if [[ "$#" -lt 2 || "$#" -gt 3 ]]; then
  echo 'Usage: bash verify_speed_improvement.sh PLANNER_YAML CONTROLLER_YAML [BASELINE_MS]' >&2
  exit 2
fi
source env.sh
export LAB4_PLANNER_PROFILE="$(realpath "$1")"
controller="$(realpath "$2")"
baseline_ms="${3:-3894.973}"
export LAB4_SIM_HEADLESS=0 LAB4_EXIT_OBSERVATION=65
unset LAB4_KEEP_FINAL_HOLD
mkdir -p results
exec 9>results/.sub7_race.lock
flock -n 9 || { echo 'A flight is already running'; exit 2; }
batch="qualify_speed_$(date +%Y%m%d_%H%M%S_%N)"
manifest="$PWD/results/${batch}_frozen.json"
python3 - "$LAB4_PLANNER_PROFILE" "$controller" "$manifest" <<'FREEZE'
import hashlib,json,sys
from pathlib import Path
r=Path.cwd()
paths=[Path(sys.argv[1]),Path(sys.argv[2]),Path.home()/'vnav-builds/lab4/lab4.x86_64',
       r/'env.sh',
       r/'code/controller_pkg/src/controller_node.cpp',
       r/'code/trajectory_generation_pkg/src/trajectory_generation_node.cpp',
       r/'code/trajectory_generation_pkg/CMakeLists.txt',
       r/'optimization/official_gates.csv',r/'optimization/race_acceptance.py',
       r/'scripts/check_sub4_qualification.py',r/'scripts/check_speed_improvement.py',
       r/'optimization/trajectory_screen.py',
       r/'code/tesse-ros-bridge/ROS/src/tesse_ros_bridge/quadrotor_control_interface.py',
       r/'code/tesse-ros-bridge/ROS/src/tesse_ros_bridge/persistent_metadata.py',
       r/'scripts/prepare_simulator.py',r/'optimization/run_trial.sh',r/'optimization/speed_trials.py',
       *sorted((r/'code/controller_pkg/include/controller_pkg').glob('*.h')),
       r/'ros_ws/devel/.private/controller_pkg/lib/controller_pkg/controller_node',
       r/'ros_ws/devel/.private/trajectory_generation/lib/trajectory_generation/trajectory_generation_node']
data={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
Path(sys.argv[3]).write_text(json.dumps(data,indent=2)+'\n')
FREEZE
duration=$(python3 -c 'import yaml,sys; print(sum(yaml.safe_load(open(sys.argv[1]))["segment_durations"]))' "$LAB4_PLANNER_PROFILE")
for index in 1 2 3; do
  python3 - "$manifest" <<'CHECK_FROZEN'
import hashlib,json,sys
from pathlib import Path
for path,digest in json.loads(Path(sys.argv[1]).read_text()).items():
    if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:
        raise SystemExit('Qualification input changed: '+path)
CHECK_FROZEN
  prefix="${batch}_${index}"
  echo "START independent qualification $index/3: $prefix"
  mkdir -p "results/${prefix}_startup"
  if ! python3 -u scripts/prepare_simulator.py "$PWD/results/${prefix}_startup" >"results/${prefix}_startup/dispatch.log" 2>&1 9>&-; then
    echo "Startup failed; preserve this attempt"; continue
  fi
  status=0
  bash optimization/run_trial.sh "$controller" "$duration" "$prefix" 2.67 75 >"results/${prefix}_dispatch.log" 2>&1 9>&- || status=$?
  tail -5 "results/${prefix}_dispatch.log"
  if [[ -f "results/${prefix}_01/race_acceptance.json" ]]; then
    python3 scripts/audit_race_numerics.py "results/${prefix}_01" >"results/${prefix}_numerics.log" 2>&1 || true
    python3 scripts/check_speed_improvement.py "results/${prefix}_01" --baseline-ms "$baseline_ms" || true
  else
    echo "Incomplete evidence; dispatch exit code $status"
  fi
done
python3 - "$batch" "$baseline_ms" <<'SUMMARY'
import json,sys
from pathlib import Path
root=Path('results');batch=sys.argv[1];attempts=[]
for index in (1,2,3):
    p=root/(batch+'_'+str(index)+'_01')/'speed_improvement_qualification.json'
    attempts.append(json.loads(p.read_text()) if p.exists() else {'run_id':p.parent.name,'qualified':False,'reason':'missing complete evidence'})
summary={'batch':batch,'baseline_ms':float(sys.argv[2]),'requested_improvement_ms':500.,
         'rule':'3/3 fresh visible Unity flights, original gates/physics/time, actual race <= baseline minus 500 ms, no mission/exit faults or collisions, independent numeric audit, continuous stable hold >=60s',
         'attempts':attempts,'qualified':all(a['qualified'] for a in attempts)}
(root/(batch+'_summary.json')).write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2))
raise SystemExit(0 if summary['qualified'] else 1)
SUMMARY
