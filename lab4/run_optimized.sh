#!/usr/bin/env bash
# Default: race in the actual Unity 3D window. RViz is optional.
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source env.sh
export LAB4_SIM_HEADLESS="${LAB4_SIM_HEADLESS:-0}"
view="${LAB4_VIEW:-0}"
for argument in "$@"; do
  case "$argument" in
    --no-view) view=0 ;;
    --unity-window) export LAB4_SIM_HEADLESS=0 ;;
    --headless) export LAB4_SIM_HEADLESS=1 ;;
    --rviz) view=1 ;;
    *) echo "Usage: bash run_optimized.sh [--unity-window|--headless] [--rviz|--no-view]" >&2; exit 2 ;;
  esac
done
mkdir -p results
exec 9> results/.sub7_race.lock
if ! flock -n 9; then
  echo 'Another optimized run is active. Stop that terminal with Ctrl+C first.' >&2
  exit 2
fi
prefix="${LAB4_RUN_PREFIX:-sub4}_$(date +%Y%m%d_%H%M%S_%N)"
viewer_pid=''
flight_pid=''
prepare_pid=''
cleanup_view() {
  trap '' INT TERM
  if [[ -n "$prepare_pid" ]]; then
    kill -TERM "$prepare_pid" 2>/dev/null || true
    wait "$prepare_pid" 2>/dev/null || true
  fi
  if [[ -n "$flight_pid" ]]; then
    kill -TERM "$flight_pid" 2>/dev/null || true
    wait "$flight_pid" 2>/dev/null || true
  fi
  if [[ -n "$viewer_pid" ]]; then
    kill -INT "$viewer_pid" 2>/dev/null || true
    wait "$viewer_pid" 2>/dev/null || true
  fi
}
trap cleanup_view EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
mkdir -p "results/${prefix}_startup"
python3 -u scripts/prepare_simulator.py "$PWD/results/${prefix}_startup" 9>&- &
prepare_pid=$!
wait "$prepare_pid"
prepare_pid=''
if [[ "$view" == 1 ]]; then
  roslaunch controller_pkg lab4_visualization.launch     rviz_config:="$PWD/code/controller_pkg/rviz/lab4_topdown.rviz"     > "results/${prefix}_view.log" 2>&1 9>&- &
  viewer_pid=$!
fi
export LAB4_PLANNER_PROFILE="${LAB4_PLANNER_PROFILE:-$PWD/optimization/planner_profiles/sub4.yaml}"
controller_profile="${LAB4_CONTROLLER_PROFILE:-$PWD/optimization/profiles/sub4.yaml}"
export LAB4_EXIT_OBSERVATION="${LAB4_EXIT_OBSERVATION:-8}"
export LAB4_KEEP_FINAL_HOLD=1
export LAB4_SHOW_TIMER=1
printf '
Race timing: actual gate 1 to actual gate 12. Planned duration includes approach and braking.
'
printf 'Logs: %s/results/%s_01

' "$PWD" "$prefix"
duration=$(python3 -c 'import sys,yaml; print(sum(yaml.safe_load(open(sys.argv[1]))["segment_durations"]))' "$LAB4_PLANNER_PROFILE")
# Keep the lock only in this launcher, never in persistent child processes.
bash optimization/run_trial.sh "$controller_profile" "$duration" "$prefix" 2.67 75 9>&- &
flight_pid=$!
wait "$flight_pid"
flight_pid=''
