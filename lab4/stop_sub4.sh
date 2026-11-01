#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source env.sh
python3 scripts/prepare_simulator.py --stop-flight
