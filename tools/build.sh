#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$ROOT/tools/environment.sh"
export CMAKE_BUILD_PARALLEL_LEVEL=2
colcon --log-base "$LIUO_TOOL_LOG/messages" build --base-paths "$ROOT/../nlink_parser_uwb/packages/liuo_uwb_msgs" --build-base "$LIUO_MSG_BASE/build" --install-base "$LIUO_MSG_BASE/install" --executor sequential
source "$LIUO_MSG_BASE/install/local_setup.bash"
colcon --log-base "$LIUO_TOOL_LOG/build" build --base-paths "$ROOT/packages" --build-base "$LIUO_BUILD_BASE" --install-base "$LIUO_INSTALL_BASE" --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON "$@"
