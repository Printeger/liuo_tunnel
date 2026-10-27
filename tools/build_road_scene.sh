#!/usr/bin/env bash
set -euo pipefail
export PYTHONDONTWRITEBYTECODE=1
set +u
source /opt/ros/jazzy/setup.bash
source /home/dev/ws_iap/uwb_ros2/install/setup.bash
set -u
python3 -m pip install --target /home/dev/ws_iap/road_scene_deps --no-deps trimesh==5.2.0 embreex==4.4.0.post1
cmake -S "$(dirname "$0")/../packages/liuo_tunnel_sim/road_scene/interface" -B /home/dev/ws_iap/road_scene_build
cmake --build /home/dev/ws_iap/road_scene_build -j2
