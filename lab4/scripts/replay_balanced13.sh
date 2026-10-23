#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${LAB4_WS:-$ROOT/ros_ws}"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
# Use a dedicated ROS master/terminal. Never replay commands into a live bridge.
if rosnode list 2>/dev/null | grep -Eq '^/(controller_node|trajectory_generation_node|tesse/quadrotor_control_interface|lab4_trial_orchestrator)$'; then
  printf 'Refusing replay while live flight/control nodes exist. Use a separate ROS master.\n' >&2
  exit 1
fi
exec roslaunch controller_pkg lab4_balanced13_replay.launch \
  bag:="$ROOT/data/balanced13_01/balanced13_01.bag" rate:=1.0
