#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../../../.." && pwd)"

if [[ "${CONFIGURE_HIKVISION_NETWORK:-true}" == "true" ]]; then
  "${REPO_ROOT}/start_hikvision_network.sh"
fi

"${SCRIPT_DIR}/start_remote_desktop.sh"

export DISPLAY="${DISPLAY:-:99}"

source /opt/ros/noetic/setup.bash
source "${REPO_ROOT}/rslidar_catkin_ws/devel/setup.bash"

exec roslaunch "${REPO_ROOT}/rslidar_catkin_ws/src/collector/launch/hikvision/hikvision_photo.launch" "$@"
