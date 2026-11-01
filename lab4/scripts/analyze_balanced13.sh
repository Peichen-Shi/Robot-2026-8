#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${LAB4_WS:-$ROOT/ros_ws}"
RUN="$ROOT/data/balanced13_01"
OUT="${1:-$ROOT/offline_reanalysis}"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
mkdir -p "$OUT/analysis"
python3 "$ROOT/scripts/analyze_lab4_bag.py" "$RUN/balanced13_01.bag" --output-dir "$OUT/analysis"
python3 "$ROOT/scripts/offline_bag_audit.py" "$RUN/balanced13_01.bag" "$OUT/analysis"
# Nonzero is expected for this supplied run: racing P95 > 1.25 m, not a crash.
python3 "$ROOT/scripts/check_fast_result.py" "$OUT" --limit 15
