#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export LAB4_PLANNER_PROFILE="$BASE/optimization/planner_profiles/sub34.yaml"
export LAB4_CONTROLLER_PROFILE="$BASE/optimization/profiles/sub34.yaml"
export LAB4_RUN_PREFIX=sub34
export LAB4_EXIT_OBSERVATION=65
printf '\nLab4: actual gate 1 to gate 12 timing, original twelve-gate Unity scene.\n'
exec bash "$BASE/run_optimized.sh" --unity-window --no-view "$@"
