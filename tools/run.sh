#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$ROOT/tools/environment.sh"
source "$LIUO_MSG_BASE/install/local_setup.bash"
source "$LIUO_INSTALL_BASE/local_setup.bash"
exec python3 "$ROOT/tools/run.py" "$@"
