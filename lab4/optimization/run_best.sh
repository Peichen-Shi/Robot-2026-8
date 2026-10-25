#!/usr/bin/env bash
# TESSE + the collision-enabled bridge must already be running. No clock scaling.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
exec bash "$HERE/run_trial.sh" "$HERE/profiles/balanced_tau01.yaml" 12.5 \
  "speed_v1_$(date +%Y%m%d_%H%M%S)" .7 75
