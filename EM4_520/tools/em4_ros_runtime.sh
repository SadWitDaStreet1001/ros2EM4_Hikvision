#!/usr/bin/env bash

EM4_ROS_RUNTIME_ENV="${EM4_ROS_RUNTIME_ENV:-/tmp/em4_ros_runtime.env}"
EM4_ROS_MASTER_URI="${EM4_ROS_MASTER_URI:-http://127.0.0.1:11313}"
EM4_ROS_MASTER_PROBE_TIMEOUT="${EM4_ROS_MASTER_PROBE_TIMEOUT:-4}"
EM4_ROS_MASTER_PORT_START="${EM4_ROS_MASTER_PORT_START:-11311}"
EM4_ROS_MASTER_PORT_END="${EM4_ROS_MASTER_PORT_END:-11320}"

em4_can_write_dir() {
  local dir="$1"
  local test_file

  if mkdir -p "$dir" 2>/dev/null; then
    test_file="$dir/.em4_write_test"
    if (: >"$test_file") 2>/dev/null; then
      rm -f "$test_file"
      return 0
    fi
  fi
  return 1
}

em4_ensure_ros_runtime_dirs() {
  local ros_home="${ROS_HOME:-$HOME/.ros}"
  local log_dir="${ROS_LOG_DIR:-}"
  local fallback_ros_home="/tmp/em4_ros_home"
  local fallback_log_dir="/tmp/em4_ros_logs"

  if em4_can_write_dir "$ros_home"; then
    export ROS_HOME="$ros_home"
  else
    mkdir -p "$fallback_ros_home"
    export ROS_HOME="$fallback_ros_home"
    echo "[em4_ros_runtime] ROS home is not writable: $ros_home" >&2
    echo "[em4_ros_runtime] using ROS_HOME=$ROS_HOME" >&2
  fi

  if [[ -z "$log_dir" ]]; then
    log_dir="$ROS_HOME/log"
  fi

  if em4_can_write_dir "$log_dir"; then
    export ROS_LOG_DIR="$log_dir"
  else
    mkdir -p "$fallback_log_dir"
    export ROS_LOG_DIR="$fallback_log_dir"
    echo "[em4_ros_runtime] ROS log directory is not writable: $log_dir" >&2
    echo "[em4_ros_runtime] using ROS_LOG_DIR=$ROS_LOG_DIR" >&2
  fi
}

em4_probe_ros_master() {
  local uri="$1"
  timeout "$EM4_ROS_MASTER_PROBE_TIMEOUT" python3 - "$uri" <<'PY'
import errno
import http.client
import socket
import sys
import urllib.parse
import xmlrpc.client

uri = sys.argv[1]
parsed = urllib.parse.urlparse(uri)
host = parsed.hostname
port = parsed.port or 80

if parsed.scheme != "http" or not host:
    sys.exit(2)

try:
    sock = socket.create_connection((host, port), timeout=1.0)
    sock.close()
except OSError as exc:
    if getattr(exc, "errno", None) in (
        errno.ECONNREFUSED,
        errno.EHOSTUNREACH,
        errno.ENETUNREACH,
    ):
        sys.exit(1)
    sys.exit(2)

class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=2.0)

try:
    code, _message, _pid = xmlrpc.client.ServerProxy(
        uri, transport=TimeoutTransport(), allow_none=True
    ).getPid("/em4_ros_runtime_probe")
except Exception:
    sys.exit(2)

sys.exit(0 if code == 1 else 2)
PY
}

em4_master_has_topic() {
  local uri="$1"
  local topic="$2"
  timeout "$EM4_ROS_MASTER_PROBE_TIMEOUT" python3 - "$uri" "$topic" <<'PY'
import http.client
import sys
import xmlrpc.client

uri = sys.argv[1]
topic = sys.argv[2]

class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=2.0)

try:
    code, _message, state = xmlrpc.client.ServerProxy(
        uri, transport=TimeoutTransport(), allow_none=True
    ).getSystemState("/em4_ros_runtime_probe")
except Exception:
    sys.exit(2)

if code != 1:
    sys.exit(2)

publishers = state[0]
for name, nodes in publishers:
    if name == topic and nodes:
        sys.exit(0)

sys.exit(1)
PY
}

em4_write_runtime_env() {
  local tmp="${EM4_ROS_RUNTIME_ENV}.tmp"

  umask 077
  {
    printf 'export ROS_MASTER_URI=%q\n' "${ROS_MASTER_URI:-}"
    printf 'export ROS_HOME=%q\n' "${ROS_HOME:-}"
    printf 'export ROS_LOG_DIR=%q\n' "${ROS_LOG_DIR:-}"
  } >"$tmp"
  mv "$tmp" "$EM4_ROS_RUNTIME_ENV"
}

em4_source_runtime_env_if_valid() {
  local required_topic="${1:-}"

  [[ -r "$EM4_ROS_RUNTIME_ENV" ]] || return 1
  # shellcheck disable=SC1090
  source "$EM4_ROS_RUNTIME_ENV"
  [[ -n "${ROS_MASTER_URI:-}" ]] || return 1
  [[ "$ROS_MASTER_URI" == "$EM4_ROS_MASTER_URI" ]] || return 1
  em4_probe_ros_master "$ROS_MASTER_URI" || return 1

  if [[ -n "$required_topic" ]]; then
    em4_master_has_topic "$ROS_MASTER_URI" "$required_topic" || return 1
  fi
}

em4_select_master_with_topic() {
  local topic="$1"

  if [[ -n "${ROS_MASTER_URI:-}" ]] && em4_probe_ros_master "$ROS_MASTER_URI" && em4_master_has_topic "$ROS_MASTER_URI" "$topic"; then
    return 0
  fi

  export ROS_MASTER_URI="$EM4_ROS_MASTER_URI"
  em4_probe_ros_master "$ROS_MASTER_URI" || return 1
  em4_master_has_topic "$ROS_MASTER_URI" "$topic" || return 1

  return 0
}
