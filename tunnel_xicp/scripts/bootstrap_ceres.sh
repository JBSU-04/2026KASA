#!/usr/bin/env bash
set -euo pipefail
task_pkg_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
task_ws_dir="$(cd -- "$task_pkg_dir/../.." && pwd)"
cmake -S "$task_pkg_dir/third_party/ceres" -B "$task_ws_dir/build_xicp_ceres" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$task_ws_dir/.xicp_deps" \
  -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF -DCXSPARSE=OFF -DLAPACK=OFF \
  -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF -DBUILD_SHARED_LIBS=ON
cmake --build "$task_ws_dir/build_xicp_ceres" -j2
cmake --install "$task_ws_dir/build_xicp_ceres"
