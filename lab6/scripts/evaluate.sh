#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
root="$script_dir"
[[ -d "$root/code/lab6" ]] || root="$(dirname "$script_dir")"
source "$root/ws/devel/setup.bash"
export LD_LIBRARY_PATH="${LAB5_ROOT:-$HOME/lab5_complete}/opencv/lib:${LD_LIBRARY_PATH:-}"
default_bag="$root/data/office_verified_prefix.bag"
if [[ -r "$root/data/office_extended.bag" ]]; then
  default_bag="$root/data/office_extended.bag"
fi
bag=${1:-$default_bag}
if [[ ! -s "$bag" ]]; then
  printf 'Missing or empty input: %s\n' "$bag" >&2
  exit 2
fi
printf 'Evaluating: %s\n' "$bag"
out=${2:-$root/results/reproduce_$(date +%Y%m%d_%H%M%S_%N)}
mkdir -p "$out"
rosrun lab_6 pose_estimation "$bag" "$out" | tee "$out/evaluate.log"
printf 'Results: %s\n' "$out"
