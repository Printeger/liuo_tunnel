#!/usr/bin/env bash
set -euo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${LIUO_CLEAN_ENV:-0} != 1 ]]; then
 exec env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/cuda/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 LIUO_CLEAN_ENV=1 bash "$0" "$@"
fi
set +u
source /opt/ros/jazzy/setup.bash
set -u
export CMAKE_BUILD_PARALLEL_LEVEL=2
cd "$ROOT"
colcon --log-base "$ROOT/log/build" build --base-paths "$ROOT/packages" --build-base "$ROOT/build" --install-base "$ROOT/install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON "$@"
