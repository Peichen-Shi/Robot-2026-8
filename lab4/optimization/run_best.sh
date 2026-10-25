#!/usr/bin/env bash
# TESSE + the collision-enabled bridge must already be running. No clock scaling.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# Repeated complete-course trials: airborne preparation is before recording.
exec bash "$HERE/run_trial.sh" "$HERE/profiles/balanced_tilt72.yaml" 12 \
  "speed_v3_$(date +%Y%m%d_%H%M%S)" 2.67 75
