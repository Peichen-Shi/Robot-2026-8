#!/usr/bin/env bash
# TESSE + the collision-enabled bridge must already be running. No clock scaling.
set -eo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
# Until a faster profile passes repeat validation, preserve the verified fallback.
exec bash "$HERE/run_trial.sh" "$HERE/profiles/balanced_baseline.yaml" 13 \
  "speed_fallback_$(date +%Y%m%d_%H%M%S)" .7 75
