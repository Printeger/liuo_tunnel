#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
if [[ ${LIUO_CLEAN_ENV:-0} != 1 ]]; then
 exec env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 LIUO_RUN_ROOT="${LIUO_RUN_ROOT:-$ROOT/log}" LIUO_CLEAN_ENV=1 bash "$0" "$@"
fi
source /opt/ros/jazzy/setup.bash
source "$ROOT/install/local_setup.bash"
exec python3 "$ROOT/tools/run.py" "$@"
