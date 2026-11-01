#!/usr/bin/env bash
# Race in Unity's rendered 3D scene; omit RViz to reserve graphics resources.
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$BASE/run_optimized.sh" --unity-window --no-view "$@"
