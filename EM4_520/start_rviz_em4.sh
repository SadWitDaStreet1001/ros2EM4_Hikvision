#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/workspace/code/EM4_520"
WS_DIR="$ROOT_DIR/rslidar_catkin_ws"
RVIZ_CONFIG="$ROOT_DIR/EM4-driver&SDK/rslidar_sdk/rviz/rviz.rviz"
ROS_HELPER="$ROOT_DIR/tools/em4_ros_runtime.sh"
REMOTE_DESKTOP_SCRIPT="$ROOT_DIR/rslidar_catkin_ws/src/collector/scripts/remote_desktop/start_remote_desktop.sh"

display_works() {
  local display="$1"
  [[ -n "$display" ]] || return 1
  command -v xdpyinfo >/dev/null 2>&1 || return 0
  timeout 2s xdpyinfo -display "$display" >/dev/null 2>&1
}

find_working_display() {
  local socket display_num display
  for socket in /tmp/.X11-unix/X*; do
    [[ -S "$socket" ]] || continue
    display_num="${socket##*/X}"
    display=":$display_num"
    if display_works "$display"; then
      printf '%s\n' "$display"
      return 0
    fi
  done
  return 1
}

ensure_display() {
  if [[ "${EM4_RVIZ_USE_NOVNC:-0}" == "1" ]]; then
    "$REMOTE_DESKTOP_SCRIPT"
    export DISPLAY=":99"
    return
  fi

  if display_works "${DISPLAY:-}"; then
    return
  fi

  if [[ -n "${DISPLAY:-}" ]]; then
    echo "[start_rviz_em4] DISPLAY=$DISPLAY is not reachable" >&2
  else
    echo "[start_rviz_em4] DISPLAY is empty" >&2
  fi

  if new_display="$(find_working_display)"; then
    export DISPLAY="$new_display"
    echo "[start_rviz_em4] using reachable DISPLAY=$DISPLAY"
    return
  fi

  echo "[start_rviz_em4] no reachable X display found" >&2
  echo "[start_rviz_em4] if you want noVNC/Xvfb, run:" >&2
  echo "  EM4_RVIZ_USE_NOVNC=1 $0" >&2
  echo "[start_rviz_em4] or start the remote desktop first:" >&2
  echo "  $REMOTE_DESKTOP_SCRIPT" >&2
  exit 1
}

ensure_display

export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-${UID}}"
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR" 2>/dev/null || true

if command -v systemd-detect-virt >/dev/null 2>&1; then
  virt="$(systemd-detect-virt --container 2>/dev/null || true)"
  if [[ -n "$virt" ]]; then
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
  fi
fi

set +u
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
set -u

if [[ -r "$ROS_HELPER" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_HELPER"
  em4_ensure_ros_runtime_dirs
  if em4_source_runtime_env_if_valid "/rslidar_points"; then
    echo "[start_rviz_em4] using ROS_MASTER_URI=$ROS_MASTER_URI"
  else
    export ROS_MASTER_URI="${EM4_ROS_MASTER_URI:-http://127.0.0.1:11313}"
    echo "[start_rviz_em4] using fixed ROS_MASTER_URI=$ROS_MASTER_URI"
  fi
fi

exec rviz -d "$RVIZ_CONFIG"
