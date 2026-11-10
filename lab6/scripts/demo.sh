#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
root="$script_dir"
[[ -d "$root/code/lab6" ]] || root="$(dirname "$script_dir")"
method=${1:-ARUN3}
case "$method" in NISTER5|EIGHT8|KNOWN2|ARUN3|NISTER5_RAW) ;; *) echo 'Use NISTER5, EIGHT8, KNOWN2, ARUN3 or NISTER5_RAW' >&2; exit 2 ;; esac
default_bag="$root/data/office_verified_prefix.bag"
default_results="$root/results/office_preview"
if [[ -r "$root/data/office_extended.bag" && -s "$root/results/office_extended/${method}_poses.bag" ]]; then
  default_bag="$root/data/office_extended.bag"
  default_results="$root/results/office_extended"
fi
bag=${2:-$default_bag}
results=${3:-$default_results}
pose_bag="$results/${method}_poses.bag"
for input in "$bag" "$pose_bag"; do
  if [[ ! -s "$input" ]]; then
    printf 'Missing or empty input: %s\n' "$input" >&2
    exit 2
  fi
done
printf 'RGB-D input: %s\nPose input: %s\n' "$bag" "$pose_bag"
source "$root/ws/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11327
export DISABLE_ROS1_EOL_WARNINGS=1
exec roslaunch lab_6 video_tracking.launch path_to_dataset:="$bag" pose_bag:="$pose_bag"
