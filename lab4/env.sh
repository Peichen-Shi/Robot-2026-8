#!/usr/bin/env bash
LAB4_BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export LAB4_BASE
# Small matrices and virtual graphics should not oversubscribe this 2-vCPU VM.
# These limits affect host worker pools, not simulator physics or ROS time.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 LP_NUM_THREADS=1
source "$LAB4_BASE/ros_ws/devel/setup.bash"
export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1
unset ROS_HOSTNAME
export PYTHONPATH="$LAB4_BASE/code/tesse-ros-bridge/tesse-interface/src:${PYTHONPATH:-}"
export LAB4_SPEED_RESULTS="$LAB4_BASE/results"
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority XDG_RUNTIME_DIR=/run/user/1000
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
