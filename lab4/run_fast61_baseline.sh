#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export LAB4_PLANNER_PROFILE="$BASE/optimization/planner_profiles/sub7.yaml"
export LAB4_CONTROLLER_PROFILE="$BASE/optimization/profiles/sub7.yaml"
export LAB4_RUN_PREFIX=fast61_baseline
exec bash "$BASE/run_optimized.sh" --unity-window --no-view "$@"
