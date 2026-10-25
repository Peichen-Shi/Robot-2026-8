#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
BASE="${LAB4_BASE:-$(cd "$HERE/.." && pwd)}"
OUT="${LAB4_SPEED_RESULTS:-$BASE/results}"
PROFILE="$1"; DURATION="$2"; PREFIX="$3"; HEIGHT="${4:-.7}"; HOLD="${5:-75}"
source "$BASE/ros_ws/devel/setup.bash"
export LAB4_ANALYSIS_SCRIPTS="$BASE/scripts"
export PYTHONPATH="$BASE/code/tesse-ros-bridge/tesse-interface/src:${PYTHONPATH:-}"
# A crashed process can leave a dead ROS master registration. Check the node
# endpoint before refusing a flight; remove only verified-dead flight entries.
python3 - <<'PYCODE'
import rosgraph, rosnode
master=rosgraph.Master('/lab4_flight_precheck')
names=set(rosnode.get_node_names())
dead=[]
for name in ('/controller_node','/trajectory_generation_node','/lab4_trial_orchestrator'):
    if name not in names:
        continue
    if rosnode.rosnode_ping(name,max_count=1,verbose=False):
        raise SystemExit('An existing live flight process is active: '+name)
    dead.append(name)
if dead:
    rosnode.cleanup_master_blacklist(master,dead)
PYCODE
# A previous candidate's private parameters must not leak into the next trial.
rosparam delete /trajectory_generation_node 2>/dev/null || true
if [[ -n "${LAB4_PLANNER_PROFILE:-}" ]]; then
  rosparam load "$LAB4_PLANNER_PROFILE" /trajectory_generation_node
fi
rosparam delete /controller_node 2>/dev/null || true
rosparam load "$PROFILE" /controller_node
mkdir -p "$OUT"
rosrun controller_pkg controller_node >"$OUT/${PREFIX}_controller.log" 2>&1 &
CONTROL_PID=$!
TIMER_PID=''
TRIAL_PID=''
cleanup() {
  trap '' INT TERM
  if [[ -n "$TRIAL_PID" ]]; then
    # GNU timeout owns a separate process group. Signal that entire group;
    # otherwise its final-hover Python process can outlive this launcher.
    kill -TERM -- "-$TRIAL_PID" 2>/dev/null || true
    for ((attempt=0; attempt<100; attempt++)); do
      kill -0 "$TRIAL_PID" 2>/dev/null || break
      sleep .1
    done
    kill -KILL -- "-$TRIAL_PID" 2>/dev/null || true
    wait "$TRIAL_PID" 2>/dev/null || true
  fi
  if [[ -n "$TIMER_PID" ]]; then kill -TERM "$TIMER_PID" 2>/dev/null || true; wait "$TIMER_PID" 2>/dev/null || true; fi
  # Background Bash jobs can inherit SIGINT=ignored; TERM also works there.
  kill -TERM "$CONTROL_PID" 2>/dev/null || true
  wait "$CONTROL_PID" 2>/dev/null || true
  python3 - <<'STOP_MOTORS' || true
from tesse.env import Env
from tesse.msgs import PropSpeeds
Env(simulation_ip='127.0.0.1',own_ip='127.0.0.1').send(PropSpeeds(0,0,0,0))
STOP_MOTORS
}
trap cleanup EXIT
trap 'exit 130' INT TERM
sleep .5
if [[ "${LAB4_SHOW_TIMER:-0}" == 1 ]]; then
  python3 -u "$BASE/scripts/lab4_gate_timer.py" --output "$OUT/${PREFIX}_timing.json" > >(tee "$OUT/${PREFIX}_timing.log") 2>&1 &
else
  python3 -u "$BASE/scripts/lab4_gate_timer.py" --output "$OUT/${PREFIX}_timing.json" >"$OUT/${PREFIX}_timing.log" 2>&1 &
fi
TIMER_PID=$!
extra_args=()
if [[ "${LAB4_KEEP_FINAL_HOLD:-0}" == 1 ]]; then
  extra_args+=(--keep-final-hold)
  trial_timeout=86400s
else
  trial_timeout=210s
fi
timeout --signal=INT --kill-after=15s "$trial_timeout" python3 -u "$HERE/speed_trials.py" \
  --output-root "$OUT" --count 1 --first-index 1 --run-prefix "$PREFIX" \
  --target-duration "$DURATION" --terminal-segment 1.8 --time-scale 1 \
  --ready-hold .5 --start-height "$HEIGHT" --settle-seconds 5 \
  --post-hold "$HOLD" --exit-observation "${LAB4_EXIT_OBSERVATION:-5}" \
  --race-scoring --acceptance-limit 15 "${extra_args[@]}" &
TRIAL_PID=$!
wait "$TRIAL_PID"
TRIAL_PID=''
