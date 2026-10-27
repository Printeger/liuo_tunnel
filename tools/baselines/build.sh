#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$ROOT/tools/environment.sh"
cd "$ROOT"
algorithm=${1:-fastlio}
case "$algorithm" in
  fastlio) target=fast_lio ;;
  liosam) target=lio_sam ;;
  *) echo "expected fastlio or liosam" >&2; exit 2 ;;
esac
python3 tools/baselines/prepare.py --algorithm "$algorithm"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "$LIUO_DATA_ROOT/baselines/colcon" build --base-paths "$LIUO_DATA_ROOT/baselines/sources/$algorithm" "$ROOT/tools/baselines/livox_ros_driver2" --packages-up-to "$target" --build-base "$LIUO_DATA_ROOT/baselines/build" --install-base "$LIUO_DATA_ROOT/baselines/install" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
