#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ ${LIUO_BASELINE_ENV:-0} != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 LIUO_BASELINE_ENV=1 bash "$0" "$@"
fi
source /opt/ros/jazzy/setup.bash
cd "$ROOT"
python3 tools/baselines/prepare.py
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base log/baseline_colcon build --base-paths log/baseline_sources/fastlio log/baseline_sources/liosam tools/baselines/livox_ros_driver2 --build-base log/baseline_build --install-base log/baseline_install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
