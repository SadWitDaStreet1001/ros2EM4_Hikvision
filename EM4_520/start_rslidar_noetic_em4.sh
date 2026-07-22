#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/workspace/code/EM4_520"
WS_DIR="$ROOT_DIR/rslidar_catkin_ws"
CONFIG_PATH="$ROOT_DIR/EM4-driver&SDK/rslidar_sdk/config/config_em4_520.yaml"
HELPER_PATH="$ROOT_DIR/tools/configure_lidar_iface.py"
LIDAR_IFACE="lidar0"
LIDAR_MAC="20:7b:d5:1a:07:0a"
LIDAR_HOST_IP="192.168.1.102/24"
ORIGINAL_ROS_MASTER_URI="${ROS_MASTER_URI-}"
EM4_ROS_MASTER_URI="${EM4_ROS_MASTER_URI:-http://127.0.0.1:11313}"
EM4_ALLOW_EXTERNAL_ROS_MASTER_URI="${EM4_ALLOW_EXTERNAL_ROS_MASTER_URI:-0}"
ROS_MASTER_PROBE_TIMEOUT="${ROS_MASTER_PROBE_TIMEOUT:-4}"
ROS_RUNTIME_ENV_PATH="${EM4_ROS_RUNTIME_ENV:-/tmp/em4_ros_runtime.env}"

if [[ ${EUID} -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    exec sudo -E "$0" "$@"
  fi
  echo "[start_rslidar_noetic_em4] please run as root" >&2
  exit 1
fi

find_iface_by_mac() {
  local want_mac="$1"
  local iface
  for path in /sys/class/net/*; do
    iface="$(basename "$path")"
    if [[ -r "$path/address" ]] && [[ "$(cat "$path/address")" == "$want_mac" ]]; then
      printf '%s\n' "$iface"
      return 0
    fi
  done
  return 1
}

iface_ready() {
  local iface="$1"
  [[ -r "/sys/class/net/$iface/address" ]] || return 1
  [[ "$(cat "/sys/class/net/$iface/address")" == "$LIDAR_MAC" ]] || return 1
  ip link show dev "$iface" 2>/dev/null | grep -q '<.*UP' || return 1
  ip -o -4 addr show dev "$iface" 2>/dev/null | awk '{print $4}' | grep -Fxq "$LIDAR_HOST_IP"
}

can_write_dir() {
  local dir="$1"
  local test_file

  if mkdir -p "$dir" 2>/dev/null; then
    test_file="$dir/.start_rslidar_write_test"
    if (: >"$test_file") 2>/dev/null; then
      rm -f "$test_file"
      return 0
    fi
  fi
  return 1
}

ensure_ros_runtime_dirs() {
  local ros_home="${ROS_HOME:-$HOME/.ros}"
  local log_dir="${ROS_LOG_DIR:-}"
  local fallback_ros_home="/tmp/em4_ros_home"
  local fallback_log_dir="/tmp/em4_ros_logs"

  if can_write_dir "$ros_home"; then
    export ROS_HOME="$ros_home"
  else
    mkdir -p "$fallback_ros_home"
    export ROS_HOME="$fallback_ros_home"
    echo "[start_rslidar_noetic_em4] ROS home is not writable: $ros_home" >&2
    echo "[start_rslidar_noetic_em4] using ROS_HOME=$ROS_HOME" >&2
  fi

  if [[ -z "$log_dir" ]]; then
    log_dir="$ROS_HOME/log"
  fi

  if can_write_dir "$log_dir"; then
    export ROS_LOG_DIR="$log_dir"
  else
    mkdir -p "$fallback_log_dir"
    export ROS_LOG_DIR="$fallback_log_dir"
    echo "[start_rslidar_noetic_em4] ROS log directory is not writable: $log_dir" >&2
    echo "[start_rslidar_noetic_em4] using ROS_LOG_DIR=$ROS_LOG_DIR" >&2
  fi
}

probe_ros_master() {
  local uri="$1"
  timeout "$ROS_MASTER_PROBE_TIMEOUT" python3 - "$uri" <<'PY'
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
    ).getPid("/start_rslidar_noetic_em4_probe")
except Exception:
    sys.exit(2)

sys.exit(0 if code == 1 else 2)
PY
}

probe_registered_rslidar_node() {
  local uri="$1"
  timeout "$ROS_MASTER_PROBE_TIMEOUT" python3 - "$uri" <<'PY'
import http.client
import sys
import xmlrpc.client

master_uri = sys.argv[1]

class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self, host):
        return http.client.HTTPConnection(host, timeout=2.0)

master = xmlrpc.client.ServerProxy(master_uri, transport=TimeoutTransport(), allow_none=True)

try:
    code, _message, state = master.getSystemState("/start_rslidar_noetic_em4_probe")
except Exception:
    sys.exit(2)

if code != 1:
    sys.exit(2)

nodes = []
for topic, publishers in state[0]:
    if topic == "/rslidar_points":
        nodes.extend(publishers)

if "/rslidar_sdk_node" not in nodes:
    sys.exit(1)

sys.exit(0)
PY
}

write_ros_runtime_env() {
  local tmp="${ROS_RUNTIME_ENV_PATH}.tmp"

  umask 077
  {
    printf 'export ROS_MASTER_URI=%q\n' "$ROS_MASTER_URI"
    printf 'export ROS_HOME=%q\n' "${ROS_HOME:-}"
    printf 'export ROS_LOG_DIR=%q\n' "${ROS_LOG_DIR:-}"
  } >"$tmp"
  mv "$tmp" "$ROS_RUNTIME_ENV_PATH"
  echo "[start_rslidar_noetic_em4] wrote ROS runtime env to $ROS_RUNTIME_ENV_PATH"
}

prepare_ros_master_uri() {
  if [[ -n "$ORIGINAL_ROS_MASTER_URI" && "$ORIGINAL_ROS_MASTER_URI" != "$EM4_ROS_MASTER_URI" && "$EM4_ALLOW_EXTERNAL_ROS_MASTER_URI" != "1" ]]; then
    echo "[start_rslidar_noetic_em4] ignoring external ROS_MASTER_URI=$ORIGINAL_ROS_MASTER_URI" >&2
    echo "[start_rslidar_noetic_em4] EM4 uses fixed ROS_MASTER_URI=$EM4_ROS_MASTER_URI" >&2
  fi

  if [[ "$EM4_ALLOW_EXTERNAL_ROS_MASTER_URI" == "1" && -n "$ORIGINAL_ROS_MASTER_URI" ]]; then
    export ROS_MASTER_URI="$ORIGINAL_ROS_MASTER_URI"
  else
    export ROS_MASTER_URI="$EM4_ROS_MASTER_URI"
  fi

  local status
  if probe_ros_master "$ROS_MASTER_URI"; then
    status=0
  else
    status=$?
  fi

  case "$status" in
    0)
      echo "[start_rslidar_noetic_em4] using responsive ROS_MASTER_URI=$ROS_MASTER_URI"
      ;;
    1)
      echo "[start_rslidar_noetic_em4] no ROS master on $ROS_MASTER_URI, roslaunch will start one"
      ;;
    *)
      echo "[start_rslidar_noetic_em4] fixed EM4 ROS master is unresponsive: $ROS_MASTER_URI" >&2
      echo "[start_rslidar_noetic_em4] refusing to create another ROS master port" >&2
      echo "[start_rslidar_noetic_em4] cleanup first, then restart:" >&2
      echo "  bash $ROOT_DIR/reset_em4_ros_runtime.sh" >&2
      echo "  bash $ROOT_DIR/start_rslidar_noetic_em4.sh" >&2
      exit 1
      ;;
  esac
}

# ROS setup scripts expect some env vars to be unset initially.
set +u
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
set -u

ensure_ros_runtime_dirs
prepare_ros_master_uri
write_ros_runtime_env

if [[ "${EM4_RESTART_RSLIDAR:-0}" != "1" ]] && probe_registered_rslidar_node "$ROS_MASTER_URI"; then
  echo "[start_rslidar_noetic_em4] /rslidar_sdk_node is already registered on $ROS_MASTER_URI"
  echo "[start_rslidar_noetic_em4] not launching a duplicate node"
  echo "[start_rslidar_noetic_em4] run reset_em4_ros_runtime.sh first if you want a clean restart"
  exit 0
fi

# Keep the lidar NIC aligned with the sensor's configured destination IP.
if command -v ip >/dev/null 2>&1; then
  iface="$LIDAR_IFACE"
  if ! ip link show "$iface" >/dev/null 2>&1; then
    iface="$(find_iface_by_mac "$LIDAR_MAC")" || {
      echo "[start_rslidar_noetic_em4] lidar adapter not found, MAC=$LIDAR_MAC" >&2
      exit 1
    }
  fi

  if iface_ready "$LIDAR_IFACE"; then
    echo "[start_rslidar_noetic_em4] $LIDAR_IFACE already configured, skipping network reconfiguration"
  else
    ip link set dev "$iface" down
    if [[ "$iface" != "$LIDAR_IFACE" ]]; then
      ip link set dev "$iface" name "$LIDAR_IFACE"
    fi

    ip addr flush dev "$LIDAR_IFACE" scope global
    ip addr add "$LIDAR_HOST_IP" dev "$LIDAR_IFACE"
    ip link set dev "$LIDAR_IFACE" up
    sleep 1
  fi
  ip -br addr show "$LIDAR_IFACE"
else
  echo "[start_rslidar_noetic_em4] 'ip' command not found, using Python fallback to configure $LIDAR_IFACE" >&2
  python3 "$HELPER_PATH" "$LIDAR_IFACE" "$LIDAR_HOST_IP"
fi

exec roslaunch rslidar_sdk start.launch config_path:="$CONFIG_PATH"
