#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$BASE/verify_speed_improvement.sh" \
  "$BASE/optimization/planner_profiles/sub34.yaml" \
  "$BASE/optimization/profiles/sub34.yaml" "${1:-3894.973}"
