#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
BASE="${LAB4_BASE:-/home/joe/lab4_report_update_20261023/verification/lab4}"
OUT="${LAB4_SPEED_RESULTS:-/home/joe/lab4_speed5_20261025/results}"
PROFILE="$1"; DURATION="$2"; PREFIX="$3"; HEIGHT="${4:-.7}"; HOLD="${5:-75}"
source "$BASE/ros_ws/devel/setup.bash"
export LAB4_ANALYSIS_SCRIPTS="$BASE/scripts"
export PYTHONPATH="$BASE/code/tesse-ros-bridge/tesse-interface/src:${PYTHONPATH:-}"
if rosnode list | grep -Eq '^/(controller_node|trajectory_generation_node|lab4_trial_orchestrator)$'; then
  echo 'An existing flight process is active; refusing to overlap trials.' >&2
  exit 2
fi
rosparam load "$PROFILE" /controller_node
mkdir -p "$OUT"
rosrun controller_pkg controller_node >"$OUT/${PREFIX}_controller.log" 2>&1 &
CONTROL_PID=$!
TIMER_PID=''
cleanup() {
  if [[ -n "$TIMER_PID" ]]; then kill -INT "$TIMER_PID" 2>/dev/null || true; wait "$TIMER_PID" 2>/dev/null || true; fi
  kill -INT "$CONTROL_PID" 2>/dev/null || true
  wait "$CONTROL_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
sleep .5
python3 -u "$BASE/scripts/lab4_gate_timer.py" --output "$OUT/${PREFIX}_timing.json" >"$OUT/${PREFIX}_timing.log" 2>&1 &
TIMER_PID=$!
timeout --signal=INT --kill-after=15s 210s python3 -u "$HERE/speed_trials.py" \
  --output-root "$OUT" --count 1 --first-index 1 --run-prefix "$PREFIX" \
  --target-duration "$DURATION" --terminal-segment 1.8 --time-scale 1 \
  --ready-hold .5 --start-height "$HEIGHT" --settle-seconds 5 \
  --post-hold "$HOLD" --race-scoring --acceptance-limit 15
