#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="/workspace/code/EM4_520"
WS_DIR="$ROOT_DIR/rslidar_catkin_ws"
ROS_HELPER="$ROOT_DIR/tools/em4_ros_runtime.sh"
HIKVISION_NETWORK_SCRIPT="$ROOT_DIR/start_hikvision_network.sh"
HIKVISION_LAUNCH="$ROOT_DIR/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_all.launch"
HIKVISION_IFACE="hikvision"
HIKVISION_HOST_IP="192.168.1.103/32"
HIKVISION_CAMERA_IP="192.168.1.64"
REQUIRED_CLOUD_TOPIC="/rslidar_points"

source "$ROS_HELPER"

hikvision_network_ready() {
  ip link show dev "$HIKVISION_IFACE" 2>/dev/null | grep -q '<.*UP' || return 1
  ip -o -4 addr show dev "$HIKVISION_IFACE" 2>/dev/null | awk '{print $4}' | grep -Fxq "$HIKVISION_HOST_IP" || return 1
  ip route get "$HIKVISION_CAMERA_IP" 2>/dev/null | grep -q "dev $HIKVISION_IFACE" || return 1
  [[ -r "/sys/class/net/$HIKVISION_IFACE/carrier" ]] || return 1
  [[ "$(cat "/sys/class/net/$HIKVISION_IFACE/carrier")" == "1" ]]
}

ensure_hikvision_network() {
  if ! command -v ip >/dev/null 2>&1; then
    echo "[start_hikvision_all_em4] 'ip' command not found" >&2
    return 1
  fi

  if hikvision_network_ready; then
    echo "[start_hikvision_all_em4] $HIKVISION_IFACE network already configured"
    ip -br addr show "$HIKVISION_IFACE"
    ip route get "$HIKVISION_CAMERA_IP"
    return 0
  fi

  echo "[start_hikvision_all_em4] Hikvision network is not ready; trying $HIKVISION_NETWORK_SCRIPT"
  if ! bash "$HIKVISION_NETWORK_SCRIPT"; then
    echo "[start_hikvision_all_em4] failed to configure Hikvision network" >&2
    echo "[start_hikvision_all_em4] if the script reports carrier=0, check camera power/PoE, Ethernet cable, and adapter link lights" >&2
    echo "[start_hikvision_all_em4] if it reports Operation not permitted, run this in a privileged host/container terminal:" >&2
    echo "  bash $HIKVISION_NETWORK_SCRIPT" >&2
    return 1
  fi

  hikvision_network_ready
}

check_camera_http() {
  if ! command -v curl >/dev/null 2>&1; then
    return 0
  fi

  if timeout 6s curl --noproxy '*' --interface "$HIKVISION_IFACE" -fsSI "http://$HIKVISION_CAMERA_IP/" >/dev/null; then
    echo "[start_hikvision_all_em4] Hikvision camera HTTP check passed"
  else
    echo "[start_hikvision_all_em4] warning: Hikvision camera HTTP check failed; RTSP may also fail" >&2
  fi
}

detect_preview_default() {
  local display="${DISPLAY:-}"
  local display_num

  if [[ -z "$display" ]]; then
    printf 'false\n'
    return
  fi

  if [[ "$display" =~ ^:([0-9]+) ]]; then
    display_num="${BASH_REMATCH[1]}"
    if [[ -S "/tmp/.X11-unix/X$display_num" ]]; then
      printf 'true\n'
      return
    fi
    printf 'false\n'
    return
  fi

  printf 'true\n'
}

set +u
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
set -u

em4_ensure_ros_runtime_dirs

if em4_source_runtime_env_if_valid "$REQUIRED_CLOUD_TOPIC"; then
  echo "[start_hikvision_all_em4] using saved ROS_MASTER_URI=$ROS_MASTER_URI"
elif em4_select_master_with_topic "$REQUIRED_CLOUD_TOPIC"; then
  echo "[start_hikvision_all_em4] found fixed EM4 LiDAR ROS master: ROS_MASTER_URI=$ROS_MASTER_URI"
else
  echo "[start_hikvision_all_em4] cannot find $REQUIRED_CLOUD_TOPIC on fixed EM4 ROS master ${EM4_ROS_MASTER_URI:-}" >&2
  echo "[start_hikvision_all_em4] start the LiDAR first:" >&2
  echo "  bash $ROOT_DIR/start_rslidar_noetic_em4.sh" >&2
  exit 1
fi

em4_write_runtime_env
ensure_hikvision_network
check_camera_http

SHOW_IMAGE_PREVIEW="${EM4_SHOW_IMAGE_PREVIEW:-$(detect_preview_default)}"
SHOW_LIDAR_PREVIEW="${EM4_SHOW_LIDAR_PREVIEW:-false}"

echo "[start_hikvision_all_em4] show_image_preview=$SHOW_IMAGE_PREVIEW show_lidar_preview=$SHOW_LIDAR_PREVIEW"
echo "[start_hikvision_all_em4] launching Hikvision + LiDAR collector"

exec roslaunch "$HIKVISION_LAUNCH" \
  show_image_preview:="$SHOW_IMAGE_PREVIEW" \
  show_lidar_preview:="$SHOW_LIDAR_PREVIEW" \
  "$@"
