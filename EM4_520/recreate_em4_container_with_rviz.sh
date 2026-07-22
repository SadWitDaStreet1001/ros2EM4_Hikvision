#!/usr/bin/env bash
set -euo pipefail

OLD_NAME="${1:-EM4_test}"
TMP_NAME="${OLD_NAME}_rviz_tmp"
IMAGE="${IMAGE:-ros_noetic_local:latest}"
WORKDIR="/workspace/code/EM4_520"
HOST_USER="${HOST_USER:-${SUDO_USER:-${USER:-hyzx}}}"
HOST_HOME="${HOST_HOME:-/home/${HOST_USER}}"
HOST_CODEX_DIR="${HOST_CODEX_DIR:-${HOST_HOME}/.codex}"
HOST_VSCODE_DIR="${HOST_VSCODE_DIR:-${HOST_HOME}/.vscode-server}"
HOST_XAUTHORITY="${HOST_XAUTHORITY:-${XAUTHORITY:-${HOST_HOME}/.Xauthority}}"
DISPLAY_VALUE="${DISPLAY:-}"
RVIZ_CONFIG="${WORKDIR}/EM4-driver&SDK/rslidar_sdk/rviz/rviz.rviz"

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "missing required command: $cmd" >&2
    exit 1
  fi
}

require_path() {
  local path="$1"
  if [[ ! -e "$path" ]]; then
    echo "required path not found: $path" >&2
    exit 1
  fi
}

container_exists() {
  docker ps -a --format '{{.Names}}' | grep -Fxq "$1"
}

container_running() {
  docker ps --format '{{.Names}}' | grep -Fxq "$1"
}

cleanup_tmp() {
  command -v docker >/dev/null 2>&1 || return 0
  if container_exists "$TMP_NAME"; then
    docker rm -f "$TMP_NAME" >/dev/null
  fi
}

trap cleanup_tmp EXIT

if command -v systemd-detect-virt >/dev/null 2>&1; then
  virt="$(systemd-detect-virt --container 2>/dev/null || true)"
  if [[ -n "$virt" ]]; then
    echo "this script must be run on the Linux host desktop session, not inside container: $virt" >&2
    exit 2
  fi
fi

if [[ -z "$DISPLAY_VALUE" ]]; then
  echo "DISPLAY is empty. Run this from the Linux host desktop terminal, or pass DISPLAY explicitly." >&2
  echo "example: DISPLAY=:0 XAUTHORITY=\$HOME/.Xauthority $0 ${OLD_NAME}" >&2
  exit 1
fi

require_cmd docker
require_cmd xhost
require_path "$WORKDIR"
require_path "$HOST_XAUTHORITY"
require_path /tmp/.X11-unix

if [[ ! -S "/tmp/.X11-unix/X${DISPLAY_VALUE#:}" ]]; then
  echo "warning: expected X11 socket /tmp/.X11-unix/X${DISPLAY_VALUE#:} not found" >&2
fi

if container_exists "$TMP_NAME"; then
  docker rm -f "$TMP_NAME" >/dev/null
fi

# Allow the container's root user to connect to the host X server.
xhost +SI:localuser:root >/dev/null

RUN_ARGS=(
  -d
  --name "$TMP_NAME"
  --restart unless-stopped
  --privileged
  --network host
  --ipc host
  --gpus all
  -e "DISPLAY=${DISPLAY_VALUE}"
  -e "XAUTHORITY=/root/.Xauthority"
  -e "QT_X11_NO_MITSHM=1"
  -e "NVIDIA_DRIVER_CAPABILITIES=all"
  -e "NVIDIA_VISIBLE_DEVICES=all"
  -v "${WORKDIR}:${WORKDIR}"
  -v /dev:/dev
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v "${HOST_XAUTHORITY}:/root/.Xauthority:ro"
  -w "$WORKDIR"
)

if [[ -d "$HOST_CODEX_DIR" ]]; then
  RUN_ARGS+=(-v "${HOST_CODEX_DIR}:/root/.codex")
fi

if [[ -d "$HOST_VSCODE_DIR" ]]; then
  RUN_ARGS+=(-v "${HOST_VSCODE_DIR}:/root/.vscode-server")
fi

for proxy_var in HTTP_PROXY HTTPS_PROXY ALL_PROXY NO_PROXY http_proxy https_proxy all_proxy no_proxy; do
  if [[ -n "${!proxy_var:-}" ]]; then
    RUN_ARGS+=(-e "${proxy_var}=${!proxy_var}")
  fi
done

echo "creating container ${TMP_NAME} from ${IMAGE}"
docker run "${RUN_ARGS[@]}" "$IMAGE" bash -lc 'tail -f /dev/null' >/dev/null

echo "verifying GPU access"
docker exec "$TMP_NAME" bash -lc 'nvidia-smi >/dev/null'

echo "verifying RViz can connect to the X server"
set +e
docker exec "$TMP_NAME" bash -lc "
  source /opt/ros/noetic/setup.bash
  if [[ -f '${WORKDIR}/rslidar_catkin_ws/devel/setup.bash' ]]; then
    source '${WORKDIR}/rslidar_catkin_ws/devel/setup.bash'
  fi
  timeout 8s rviz -d '${RVIZ_CONFIG}'
" >/tmp/"${TMP_NAME}".rviz.log 2>&1
rviz_rc=$?
set -e

if [[ $rviz_rc -ne 124 ]]; then
  echo "RViz startup check failed. Log follows:" >&2
  cat /tmp/"${TMP_NAME}".rviz.log >&2
  exit 1
fi

echo "RViz startup check passed"

if container_exists "$OLD_NAME"; then
  echo "removing old container ${OLD_NAME}"
  docker rm -f "$OLD_NAME" >/dev/null
fi

echo "renaming ${TMP_NAME} -> ${OLD_NAME}"
docker rename "$TMP_NAME" "$OLD_NAME"
trap - EXIT

echo "container rebuilt successfully: ${OLD_NAME}"
echo "enter it with:"
echo "  docker exec -it ${OLD_NAME} bash"
