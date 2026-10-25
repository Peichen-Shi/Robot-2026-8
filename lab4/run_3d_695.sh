#!/usr/bin/env bash
# Retain the measured 6.95-second gate-centre version as a selectable baseline.
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export LAB4_CONTROLLER_PROFILE="$BASE/optimization/phase2/baseline_695_controller.yaml"
export LAB4_PLANNER_PROFILE="$BASE/optimization/phase2/baseline_695_timing.yaml"
exec bash "$BASE/run_3d_race.sh" "$@"
