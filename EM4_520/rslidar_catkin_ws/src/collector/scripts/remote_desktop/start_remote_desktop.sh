#!/usr/bin/env bash
set -euo pipefail

DISPLAY_NUM="${DISPLAY_NUM:-99}"
DISPLAY=":${DISPLAY_NUM}"
GEOMETRY="${GEOMETRY:-1920x1080x24}"
VNC_PORT="${VNC_PORT:-5901}"
NOVNC_PORT="${NOVNC_PORT:-6080}"
BIND_ADDR="${BIND_ADDR:-127.0.0.1}"
NOVNC_WEB="${NOVNC_WEB:-/usr/share/novnc}"
LOG_DIR="${LOG_DIR:-/tmp/collector-remote-desktop}"

XVFB_BIN="${XVFB_BIN:-Xvfb}"
X11VNC_BIN="${X11VNC_BIN:-x11vnc}"
WEBSOCKIFY_BIN="${WEBSOCKIFY_BIN:-websockify}"
OPENBOX_BIN="${OPENBOX_BIN:-openbox}"

PID_XVFB="${LOG_DIR}/xvfb-${DISPLAY_NUM}.pid"
PID_OPENBOX="${LOG_DIR}/openbox-${DISPLAY_NUM}.pid"
PID_X11VNC="${LOG_DIR}/x11vnc-${DISPLAY_NUM}.pid"
PID_WEBSOCKIFY="${LOG_DIR}/websockify-${DISPLAY_NUM}.pid"

mkdir -p "${LOG_DIR}"

is_alive() {
  local pid_file="$1"
  [ -f "$pid_file" ] && kill -0 "$(cat "$pid_file")" 2>/dev/null
}

wait_for_display() {
  local tries=0
  while ! xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; do
    tries=$((tries + 1))
    if [ "${tries}" -gt 50 ]; then
      echo "timed out waiting for ${DISPLAY}" >&2
      exit 1
    fi
    sleep 0.2
  done
}

start_xvfb() {
  if xdpyinfo -display "${DISPLAY}" >/dev/null 2>&1; then
    return
  fi

  if [ -f "/tmp/.X${DISPLAY_NUM}-lock" ]; then
    local lock_pid
    lock_pid="$(cat "/tmp/.X${DISPLAY_NUM}-lock" 2>/dev/null || true)"
    if [ -n "${lock_pid}" ] && ! kill -0 "${lock_pid}" 2>/dev/null; then
      rm -f "/tmp/.X${DISPLAY_NUM}-lock"
    fi
  fi

  if [ -S "/tmp/.X11-unix/X${DISPLAY_NUM}" ]; then
    rm -f "/tmp/.X11-unix/X${DISPLAY_NUM}"
  fi

  nohup "${XVFB_BIN}" "${DISPLAY}" -screen 0 "${GEOMETRY}" -nolisten tcp -ac \
    >"${LOG_DIR}/xvfb-${DISPLAY_NUM}.log" 2>&1 &
  echo $! >"${PID_XVFB}"

  wait_for_display
  DISPLAY="${DISPLAY}" xset s off -dpms >/dev/null 2>&1 || true
}

start_openbox() {
  if is_alive "${PID_OPENBOX}"; then
    return
  fi

  nohup env DISPLAY="${DISPLAY}" "${OPENBOX_BIN}" \
    >"${LOG_DIR}/openbox-${DISPLAY_NUM}.log" 2>&1 &
  echo $! >"${PID_OPENBOX}"
}

start_x11vnc() {
  if is_alive "${PID_X11VNC}" || ss -ltn 2>/dev/null | grep -q "[.:]${VNC_PORT}[[:space:]]"; then
    return
  fi

  nohup "${X11VNC_BIN}" \
    -display "${DISPLAY}" \
    -localhost \
    -nopw \
    -forever \
    -shared \
    -rfbport "${VNC_PORT}" \
    -noxdamage \
    -repeat \
    -quiet \
    >"${LOG_DIR}/x11vnc-${DISPLAY_NUM}.log" 2>&1 &
  echo $! >"${PID_X11VNC}"

  sleep 1
  if ! ss -ltn 2>/dev/null | grep -q "[.:]${VNC_PORT}[[:space:]]"; then
    echo "x11vnc did not start on port ${VNC_PORT}" >&2
    exit 1
  fi
}

start_websockify() {
  if is_alive "${PID_WEBSOCKIFY}" || ss -ltn 2>/dev/null | grep -q "[.:]${NOVNC_PORT}[[:space:]]"; then
    return
  fi

  nohup "${WEBSOCKIFY_BIN}" --web "${NOVNC_WEB}" "${BIND_ADDR}:${NOVNC_PORT}" "${BIND_ADDR}:${VNC_PORT}" \
    >"${LOG_DIR}/websockify-${DISPLAY_NUM}.log" 2>&1 &
  echo $! >"${PID_WEBSOCKIFY}"

  sleep 1
  if ! ss -ltn 2>/dev/null | grep -q "[.:]${NOVNC_PORT}[[:space:]]"; then
    echo "websockify did not start on port ${NOVNC_PORT}" >&2
    exit 1
  fi
}

start_xvfb
start_openbox
start_x11vnc
start_websockify

cat <<EOF
remote desktop ready
  display: ${DISPLAY}
  vnc:     ${BIND_ADDR}:${VNC_PORT}
  noVNC:   http://${BIND_ADDR}:${NOVNC_PORT}/vnc.html?autoconnect=1&resize=scale
  logs:    ${LOG_DIR}
EOF
