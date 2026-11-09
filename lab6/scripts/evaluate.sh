#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
root="$script_dir"
[[ -d "$root/code/lab6" ]] || root="$(dirname "$script_dir")"
source "$root/ws/devel/setup.bash"
export LD_LIBRARY_PATH="${LAB5_ROOT:-$HOME/lab5_complete}/opencv/lib:${LD_LIBRARY_PATH:-}"
bag=${1:-$root/data/office_verified_prefix.bag}
out=${2:-$root/results/reproduce_$(date +%Y%m%d_%H%M%S_%N)}
mkdir -p "$out"
rosrun lab_6 pose_estimation "$bag" "$out" | tee "$out/evaluate.log"
printf 'Results: %s\n' "$out"
