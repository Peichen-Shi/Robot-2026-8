#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
root="$script_dir"
[[ -d "$root/code/lab6" ]] || root="$(dirname "$script_dir")"
cd "$root"
url='https://drive.usercontent.google.com/download?id=1TohOnfjR_FKf_y9wYTEkmv-XCuFPFRPt&export=download&confirm=t'
mkdir -p data logs
if [[ "$(stat -c %s data/office_ros2.zip 2>/dev/null || echo 0)" != 8574812697 ]]; then
  curl -fL -C - --retry 4 -o data/office_ros2.zip "$url"
fi
test "$(stat -c %s data/office_ros2.zip)" = 8574812697
unzip -o data/office_ros2.zip -d data/full_archive
PYTHONPATH="$root/python_deps:${PYTHONPATH:-}" python3 -u "$script_dir/convert_office.py" \
  data/full_archive/vnav-lab6-office/vnav-lab6-office.db3 data/office_full.bag
bash "$script_dir/evaluate.sh" "$root/data/office_full.bag" "$root/results/office_full"
