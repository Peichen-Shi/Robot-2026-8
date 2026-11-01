#!/usr/bin/env bash
# Ubuntu 20.04 / ROS Noetic; catkin-tools, Eigen3, NLopt and glog are required.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${1:-$ROOT/ros_ws}"
source /opt/ros/noetic/setup.bash
mkdir -p "$WS/src"
link_source() {
  local source_path="$1" link_path="$2"
  if [[ -e "$link_path" || -L "$link_path" ]]; then
    [[ "$(readlink -f -- "$link_path")" == "$(readlink -f -- "$source_path")" ]] || {
      printf 'Refusing to replace existing workspace path: %s\n' "$link_path" >&2
      exit 1
    }
  else
    ln -s -- "$source_path" "$link_path"
  fi
}
for name in controller_pkg trajectory_generation_pkg planner_pkg dependencies; do
  link_source "$ROOT/code/$name" "$WS/src/$name"
done
link_source "$ROOT/code/tesse-ros-bridge/ROS" "$WS/src/tesse_ros_bridge"
cd "$WS"
catkin init
catkin config --extend "${LAB4_UNDERLAY:-/opt/ros/noetic}" --cmake-args -DCMAKE_BUILD_TYPE=Release
if [[ -n "${LAB4_UNDERLAY:-}" ]]; then
  # An existing Noetic underlay must already provide the vendored dependencies.
  catkin build --no-status -j2 -p1 --no-deps mav_planning_msgs mav_visualization trajectory_generation controller_pkg planner_pkg tesse_ros_bridge
else
  # The upstream glog/NLopt wrappers download their original source archives.
  catkin build --no-status -j2 -p1
fi
printf '\nBuilt workspace: %s\n' "$WS"
printf 'Source %s/devel/setup.bash before launch.\n' "$WS"
printf 'TESSE Python module path: %s/code/tesse-ros-bridge/tesse-interface/src\n' "$ROOT"
