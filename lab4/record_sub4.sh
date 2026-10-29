#!/usr/bin/env bash
set -e
BASE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$BASE/env.sh"
exec python3 "$BASE/scripts/record_sub4_video.py" "$@"
