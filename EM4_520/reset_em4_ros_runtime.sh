#!/usr/bin/env bash
EM4_RESET_SOURCED=0
EM4_RESET_SHELL_OPTIONS=""
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
  EM4_RESET_SOURCED=1
  EM4_RESET_SHELL_OPTIONS="$(set +o)"
fi
set -euo pipefail

PORTS=($(seq 11311 11320))
EM4_ROS_RUNTIME_ENV="${EM4_ROS_RUNTIME_ENV:-/tmp/em4_ros_runtime.env}"
PATTERNS=(
  "roscore"
  "rosmaster"
  "roslaunch"
  "rosout"
  "rslidar_sdk_node"
  "rviz"
  "rostopic"
  "rosnode"
  "rosbag"
  "rosparam"
  "rosservice"
  "hikvision_.*node"
  "hikvision_collector"
  "hikvision_photo"
)

is_sourced() {
  [[ "$EM4_RESET_SOURCED" -eq 1 ]]
}

finish_reset() {
  local status="${1:-0}"
  if is_sourced; then
    set +u
    eval "$EM4_RESET_SHELL_OPTIONS"
    return "$status"
  fi
  exit "$status"
}

show_ros_processes() {
  local regex
  regex="$(IFS='|'; echo "${PATTERNS[*]}")"
  ps -eo pid=,stat=,args= | awk -v re="$regex" '
    $2 !~ /^Z/ && $0 ~ re && $0 !~ /awk -v re=/ && $0 !~ /reset_em4_ros_runtime/ {print}
  '
}

kill_pattern() {
  local pattern="$1"
  pkill -TERM -f "$pattern" 2>/dev/null || true
}

kill_pattern_force() {
  local pattern="$1"
  pkill -KILL -f "$pattern" 2>/dev/null || true
}

show_ros_ports() {
  local port
  for port in "${PORTS[@]}"; do
    ss -H -ltnp "sport = :$port" 2>/dev/null || true
  done
}

has_port_listener() {
  local port="$1"
  ss -H -ltn "sport = :$port" 2>/dev/null | grep -q .
}

port_has_visible_pid() {
  local port="$1"
  ss -H -ltnp "sport = :$port" 2>/dev/null | grep -q 'users:'
}

echo "[reset_em4_ros_runtime] stopping visible ROS/EM4 runtime processes"
before="$(show_ros_processes)"
if [[ -n "$before" ]]; then
  printf '%s\n' "$before"
else
  echo "[reset_em4_ros_runtime] no visible ROS/EM4 process found"
fi

for pattern in "${PATTERNS[@]}"; do
  kill_pattern "$pattern"
done

sleep 1

for pattern in "${PATTERNS[@]}"; do
  kill_pattern_force "$pattern"
done

echo "[reset_em4_ros_runtime] checking ROS master ports"
ports_before="$(show_ros_ports)"
if [[ -n "$ports_before" ]]; then
  printf '%s\n' "$ports_before"
else
  echo "[reset_em4_ros_runtime] no visible listener on 11311-11320"
fi

blocked_ports=()
for port in "${PORTS[@]}"; do
  if has_port_listener "$port" && ! port_has_visible_pid "$port"; then
    blocked_ports+=("$port")
  fi
done

if ((${#blocked_ports[@]} > 0)); then
  echo "[reset_em4_ros_runtime] warning: port(s) still occupied without a visible PID: ${blocked_ports[*]}" >&2
  echo "[reset_em4_ros_runtime] this usually means the process is on the Linux host or another namespace" >&2
  echo "[reset_em4_ros_runtime] run this on the Linux host terminal if you want to free the default ROS port:" >&2
  echo "  pkill -f roscore" >&2
  echo "  pkill -f rosmaster" >&2
  echo "  pkill -f roslaunch" >&2
fi

rm -f "$EM4_ROS_RUNTIME_ENV" 2>/dev/null || true
echo "[reset_em4_ros_runtime] removed runtime env file: $EM4_ROS_RUNTIME_ENV"

echo "[reset_em4_ros_runtime] current ROS-related environment in this shell:"
env | sort | grep -E '^(ROS_MASTER_URI|ROS_HOSTNAME|ROS_IP|ROS_HOME|ROS_LOG_DIR)=' || true

if is_sourced; then
  unset ROS_MASTER_URI ROS_HOSTNAME ROS_IP ROS_HOME ROS_LOG_DIR
  echo "[reset_em4_ros_runtime] unset ROS_MASTER_URI/ROS_HOSTNAME/ROS_IP/ROS_HOME/ROS_LOG_DIR in the current shell"
else
  echo "[reset_em4_ros_runtime] note: run with 'source reset_em4_ros_runtime.sh' to unset ROS env vars in the current shell"
fi

echo "[reset_em4_ros_runtime] done"
finish_reset 0
