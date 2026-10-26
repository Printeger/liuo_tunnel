#!/usr/bin/env bash
# Source from launcher-protected entrypoints. Keep storage paths across env -i.
export LIUO_DATA_ROOT="${LIUO_DATA_ROOT:-/home/dev/ws_iap/liuo_observation}"
export LIUO_BUILD_BASE="${LIUO_BUILD_BASE:-$LIUO_DATA_ROOT/build}"
export LIUO_INSTALL_BASE="${LIUO_INSTALL_BASE:-$LIUO_DATA_ROOT/install}"
export LIUO_MSG_BASE="${LIUO_MSG_BASE:-$LIUO_DATA_ROOT/messages}"
export LIUO_TOOL_LOG="${LIUO_TOOL_LOG:-$LIUO_DATA_ROOT/log}"
export PYTHONDONTWRITEBYTECODE=1
if [[ ${LIUO_CLEAN_ENV:-0} != 1 ]]; then
  exec env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/local/cuda/bin:/usr/local/bin:/usr/bin:/bin LANG=C.UTF-8 \
    LIUO_CLEAN_ENV=1 LIUO_DATA_ROOT="$LIUO_DATA_ROOT" LIUO_BUILD_BASE="$LIUO_BUILD_BASE" \
    LIUO_INSTALL_BASE="$LIUO_INSTALL_BASE" LIUO_MSG_BASE="$LIUO_MSG_BASE" LIUO_TOOL_LOG="$LIUO_TOOL_LOG" \
    LIUO_RUN_ROOT="${LIUO_RUN_ROOT:-$LIUO_DATA_ROOT/runs}" LIUO_RUN_DIR="${LIUO_RUN_DIR:-}" \
    TMPDIR="${TMPDIR:-$LIUO_DATA_ROOT/tmp}" XDG_CACHE_HOME="${XDG_CACHE_HOME:-$LIUO_DATA_ROOT/cache}" \
    ROS_LOG_DIR="${ROS_LOG_DIR:-$LIUO_DATA_ROOT/ros}" PYTHONDONTWRITEBYTECODE=1 \
    DISPLAY="${DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}" bash "$0" "$@"
fi
set +u
source /opt/ros/jazzy/setup.bash
