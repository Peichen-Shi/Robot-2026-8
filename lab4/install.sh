#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export LAB4_UNDERLAY="${LAB4_UNDERLAY:-$HOME/vnav_ws4/devel}"
if [[ ! -f "$LAB4_UNDERLAY/setup.bash" ]]; then
  echo 'A built ROS Noetic dependency underlay is required. Set LAB4_UNDERLAY to its devel directory.' >&2
  exit 2
fi
bash scripts/build_workspace.sh
chmod +x code/tesse-ros-bridge/ROS/src/tesse_ros_bridge/*.py scripts/*.py
printf '
Start: bash "%s/run_optimized.sh"
' "$PWD"
