#!/usr/bin/env bash
set -eo pipefail
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source "$ROOT/tools/environment.sh"
source "$LIUO_MSG_BASE/install/local_setup.bash"
source "$LIUO_INSTALL_BASE/local_setup.bash"
cd "$ROOT"
colcon --log-base "$LIUO_TOOL_LOG/test" test --base-paths "$ROOT/packages" --build-base "$LIUO_BUILD_BASE" --event-handlers console_direct+
colcon --log-base "$LIUO_TOOL_LOG/test" test-result --test-result-base "$LIUO_BUILD_BASE" --verbose
python3 -m unittest discover -s tools -p 'test_*.py' -v
python3 tools/check_pressure_message.py
