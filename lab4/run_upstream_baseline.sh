#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source env.sh
export LAB4_SIM_HEADLESS="${LAB4_SIM_HEADLESS:-1}"
mkdir -p results
exec 9> results/.sub7_race.lock
flock -n 9 || { echo 'Another racing session is active.' >&2; exit 2; }
prefix="baseline_$(date +%Y%m%d_%H%M%S)"
mkdir -p "results/${prefix}_startup"
python3 -u scripts/prepare_simulator.py "$PWD/results/${prefix}_startup"
unset LAB4_PLANNER_PROFILE
export LAB4_KEEP_FINAL_HOLD=1
flight_pid=''
cleanup() {
  if [[ -n "$flight_pid" ]]; then
    kill -TERM "$flight_pid" 2>/dev/null || true
    wait "$flight_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
bash optimization/run_trial.sh "$PWD/optimization/profiles/fast_attitude.yaml" 10.5 "$prefix" 2.67 75 9>&- &
flight_pid=$!
wait "$flight_pid"
flight_pid=''
