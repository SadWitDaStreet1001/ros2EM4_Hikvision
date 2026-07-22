#!/usr/bin/env bash
set -euo pipefail

DISPLAY_VALUE="${DISPLAY:-:0}"
XAUTHORITY_VALUE="${XAUTHORITY:-${HOME}/.Xauthority}"

if command -v systemd-detect-virt >/dev/null 2>&1; then
  virt="$(systemd-detect-virt --container 2>/dev/null || true)"
  if [[ -n "${virt}" ]]; then
    echo "[prepare_host_x11_for_em4_container] current shell is inside a container: ${virt}" >&2
    echo "[prepare_host_x11_for_em4_container] run this script on the Linux host desktop session" >&2
    exit 2
  fi
fi

if ! command -v xhost >/dev/null 2>&1; then
  echo "[prepare_host_x11_for_em4_container] xhost not found; install x11-xserver-utils on the host" >&2
  exit 1
fi

if [[ ! -S "/tmp/.X11-unix/X${DISPLAY_VALUE#:}" ]]; then
  echo "[prepare_host_x11_for_em4_container] warning: /tmp/.X11-unix/X${DISPLAY_VALUE#:} not found" >&2
  echo "[prepare_host_x11_for_em4_container] run this from the Linux desktop terminal, not from a plain SSH session" >&2
fi

if [[ ! -e "${XAUTHORITY_VALUE}" ]]; then
  echo "[prepare_host_x11_for_em4_container] warning: XAUTHORITY file not found: ${XAUTHORITY_VALUE}" >&2
fi

xhost +SI:localuser:root >/dev/null

echo "host X11 is ready for the container"
echo "DISPLAY=${DISPLAY_VALUE}"
echo "XAUTHORITY=${XAUTHORITY_VALUE}"
echo
echo "recreate/check the container with:"
echo "  DISPLAY=${DISPLAY_VALUE} XAUTHORITY=${XAUTHORITY_VALUE} /workspace/code/EM4_520/recreate_em4_container_with_rviz.sh EM4_test"
