#!/usr/bin/env bash
set -eo pipefail
script_dir="$(cd "$(dirname "$0")" && pwd)"
root="$script_dir"
[[ -d "$root/code/lab6" ]] || root="$(dirname "$script_dir")"
lab5=${LAB5_ROOT:-$HOME/lab5_complete}
mkdir -p "$root"/{ws/src,logs}
if [[ ! -d "$root/opengv/.git" ]]; then
  git clone https://github.com/ethz-asl/opengv.git "$root/opengv"
fi
git -C "$root/opengv" checkout --detach 7542b890f4d8655119c80763795475bec4667fc9
cmake -S "$root/opengv" -B "$root/opengv-build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS="-I/usr/include/eigen3 -I/usr/include/eigen3/unsupported" \
  -DCMAKE_CXX_FLAGS_RELEASE="-O1 -DNDEBUG" -DBUILD_SHARED_LIBS=ON
cmake --build "$root/opengv-build" --target opengv -j2
ln -sfn ../../code/lab6 "$root/ws/src/lab6"
source "$lab5/ws/devel/setup.bash"
cd "$root/ws"
catkin_make -DOpenCV_DIR="$lab5/opencv/lib/cmake/opencv4" \
  -DOPENGV_ROOT="$root" -DCMAKE_BUILD_TYPE=Release -j2
