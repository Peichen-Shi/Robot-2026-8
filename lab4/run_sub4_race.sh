#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export LAB4_PLANNER_PROFILE="$BASE/optimization/planner_profiles/sub4.yaml"
export LAB4_CONTROLLER_PROFILE="$BASE/optimization/profiles/sub4.yaml"
export LAB4_RUN_PREFIX=sub4
printf '\nLab4: sub-four-second configuration, original twelve-gate Unity scene.\n'
exec bash "$BASE/run_optimized.sh" --unity-window --no-view "$@"
