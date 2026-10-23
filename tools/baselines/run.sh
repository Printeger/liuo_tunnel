#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ ${LIUO_BASELINE_ENV:-0} != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 LIUO_BASELINE_ENV=1 bash "$0" "$@"
fi
source /opt/ros/jazzy/setup.bash
source "$ROOT/log/baseline_install/local_setup.bash"
cd "$ROOT"
exec python3 tools/baselines/run_external.py "$@"
