#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${DISPLAY_NUM:-99}"
LOG_DIR="${LOG_DIR:-/tmp/collector-remote-desktop}"

stop_pid_file() {
  local pid_file="$1"
  if [ ! -f "${pid_file}" ]; then
    return
  fi

  local pid
  pid="$(cat "${pid_file}" 2>/dev/null || true)"
  if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
    kill "${pid}" 2>/dev/null || true
  fi
  rm -f "${pid_file}"
}

stop_pid_file "${LOG_DIR}/websockify-${DISPLAY_NUM}.pid"
stop_pid_file "${LOG_DIR}/x11vnc-${DISPLAY_NUM}.pid"
stop_pid_file "${LOG_DIR}/openbox-${DISPLAY_NUM}.pid"
stop_pid_file "${LOG_DIR}/xvfb-${DISPLAY_NUM}.pid"

sleep 0.5
if ! xdpyinfo -display ":${DISPLAY_NUM}" >/dev/null 2>&1; then
  rm -f "/tmp/.X${DISPLAY_NUM}-lock" "/tmp/.X11-unix/X${DISPLAY_NUM}"
fi

echo "remote desktop stopped for display :${DISPLAY_NUM}"
