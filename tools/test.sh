#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${LIUO_CLEAN_ENV:-0} != 1 ]]; then
 exec env -i HOME="$HOME" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 LIUO_CLEAN_ENV=1 bash "$0" "$@"
fi
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/local_setup.bash"
cd "$ROOT"
colcon --log-base "$ROOT/log/test" test --base-paths "$ROOT/packages" --build-base "$ROOT/build" --event-handlers console_direct+
colcon test-result --test-result-base "$ROOT/build" --verbose
python3 -m unittest discover -s tools -p 'test_*.py' -v
