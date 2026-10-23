#!/usr/bin/env bash
# Run only after starting the LAB4 TESSE binary and tesse_quadrotor_bridge.launch.
# Does not claim a new flight is identical to the recorded bag or relax precision.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${LAB4_WS:-$ROOT/ros_ws}"
OUT="${1:-$ROOT/new_runs}"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
export PYTHONPATH="$ROOT/code/tesse-ros-bridge/tesse-interface/src:${PYTHONPATH:-}"
if rosnode list | grep -Eq '^/(controller_node|trajectory_generation_node|lab4_trial_orchestrator)$'; then
  printf 'Stop the existing controller/planner/trial before starting this dedicated workflow.\n' >&2
  exit 1
fi
timeout 8s rostopic echo -n1 /current_state >/dev/null
roslaunch controller_pkg lab4_balanced13_controller.launch &
CONTROLLER_PID=$!
TIMER_PID=""
cleanup() {
  if [[ -n "$TIMER_PID" ]]; then
    kill -INT "$TIMER_PID" 2>/dev/null || true
    wait "$TIMER_PID" 2>/dev/null || true
  fi
  kill -INT "$CONTROLLER_PID" 2>/dev/null || true
  wait "$CONTROLLER_PID" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM
for _ in $(seq 1 30); do
  rosnode list | grep -qx '/controller_node' && break
  sleep .2
done
python3 - <<'PY'
import math, rospy
rospy.init_node('balanced13_parameter_guard',anonymous=True)
expected={'kx':12,'kv':7,'kr':8,'komega':2.4,'maximum_tilt_deg':65,
          'maximum_thrust':35,'feedforward_filter_tau':.03}
for key,value in expected.items():
    actual=rospy.get_param('/controller_node/'+key)
    if not math.isclose(float(actual),value,abs_tol=1e-8,rel_tol=0):
        raise RuntimeError('Parameter mismatch: '+key)
assert rospy.get_param('/controller_node/attitude_feedforward')
assert rospy.get_param('/controller_node/use_stamped_reference')
PY
python3 -u "$ROOT/scripts/lab4_gate_timer.py" \
  --output "$OUT/gate_timing_$(date +%Y%m%d_%H%M%S).json" &
TIMER_PID=$!
python3 "$ROOT/scripts/run_repeated_trials.py" \
  --output-root "$OUT" --count 1 --first-index 1 \
  --run-prefix "balanced13_reproduction_$(date +%Y%m%d_%H%M%S)" \
  --target-duration 13 --terminal-segment 1.8 --time-scale 1 \
  --ready-hold .5 --start-height .7 --settle-seconds 10 \
  --acceptance-limit 15 --keep-final-hold
# The original 1.25 m racing-P95 rule remains active and can return nonzero.
# Ctrl-C ends control; the independent bridge watchdog zeros stale commands.
