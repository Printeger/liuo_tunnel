#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
source "$ROOT/tools/environment.sh"
source "$LIUO_DATA_ROOT/baselines/install/local_setup.bash"
cd "$ROOT"
exec python3 tools/baselines/run_external.py "$@"
