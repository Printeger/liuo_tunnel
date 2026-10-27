#!/usr/bin/env bash
set -euo pipefail
# Invoke only through data-run. Dependencies and all output remain on data disk.
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="/home/dev/ws_iap/road_scene_deps:${PYTHONPATH:-}"
set +u
source /opt/ros/jazzy/setup.bash
source /home/dev/ws_iap/uwb_ros2/install/setup.bash
set -u
exec python3 "$(dirname "$0")/road_scene.py" "$@"
