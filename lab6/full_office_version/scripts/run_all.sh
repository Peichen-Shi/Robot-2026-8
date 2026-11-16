#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
source "$HOME/lab6_ws/devel/setup.bash"
result="${1:-$HOME/lab6_results}"
mkdir -p "$result"
rosrun lab_6 pose_estimation --bag "$HOME/data/vnav/office.bag" --output "$result" | tee "$result/run_all.log"
python3 "$HOME/lab6_ws/src/lab6/scripts/make_report.py" "$result"
