#!/usr/bin/env bash
# Independent prepared-start attempts; retain failures and continue the series.
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PROFILE="$1"; T="$2"; HEIGHT="$3"; N="$4"; PREFIX="$5"
OUT="${LAB4_SPEED_RESULTS:-/home/joe/lab4_speed5_20261025/results}"
for index in $(seq 1 "$N"); do
  name="${PREFIX}_$(printf '%02d' "$index")"
  echo "BEGIN $name $(date -Iseconds)"
  bash "$HERE/run_trial.sh" "$PROFILE" "$T" "$name" "$HEIGHT" 75 >"$OUT/${name}_batch.log" 2>&1
  result=$?
  echo "END $name exit=$result $(date -Iseconds)"
done
