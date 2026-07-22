#!/usr/bin/env bash
set -euo pipefail

HIKVISION_IFACE="hikvision"
HIKVISION_MAC="6c:1f:f7:df:9f:fb"
HIKVISION_HOST_ADDR="192.168.1.103"
HIKVISION_HOST_IP="${HIKVISION_HOST_ADDR}/32"
HIKVISION_CAMERA_IP="192.168.1.64"

if [[ ${EUID} -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    exec sudo -E "$0" "$@"
  fi
  echo "[start_hikvision_network] please run as root" >&2
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
  [[ "$(cat "/sys/class/net/$iface/address")" == "$HIKVISION_MAC" ]] || return 1
  ip link show dev "$iface" 2>/dev/null | grep -q '<.*UP' || return 1
  ip -o -4 addr show dev "$iface" 2>/dev/null | awk '{print $4}' | grep -Fxq "$HIKVISION_HOST_IP" || return 1
  ip route get "$HIKVISION_CAMERA_IP" 2>/dev/null | grep -q "dev $iface"
}

iface_has_carrier() {
  local iface="$1"
  [[ -r "/sys/class/net/$iface/carrier" ]] || return 1
  [[ "$(cat "/sys/class/net/$iface/carrier")" == "1" ]]
}

print_state() {
  ip -br link show "$HIKVISION_IFACE" 2>/dev/null || true
  ip -br addr show "$HIKVISION_IFACE" 2>/dev/null || true
  ip route get "$HIKVISION_CAMERA_IP" 2>/dev/null || true
  if [[ -r "/sys/class/net/$HIKVISION_IFACE/carrier" ]]; then
    echo "[start_hikvision_network] carrier=$(cat "/sys/class/net/$HIKVISION_IFACE/carrier") operstate=$(cat "/sys/class/net/$HIKVISION_IFACE/operstate" 2>/dev/null || echo unknown)"
  fi
}

if ! command -v ip >/dev/null 2>&1; then
  echo "[start_hikvision_network] 'ip' command not found" >&2
  exit 1
fi

iface="$HIKVISION_IFACE"
if ! ip link show "$iface" >/dev/null 2>&1; then
  iface="$(find_iface_by_mac "$HIKVISION_MAC")" || {
    echo "[start_hikvision_network] Hikvision USB-RJ45 adapter not found, MAC=$HIKVISION_MAC" >&2
    exit 1
  }
fi

if iface_ready "$HIKVISION_IFACE"; then
  echo "[start_hikvision_network] $HIKVISION_IFACE already configured, skipping network reconfiguration"
else
  ip link set dev "$iface" down
  if [[ "$iface" != "$HIKVISION_IFACE" ]]; then
    ip link set dev "$iface" name "$HIKVISION_IFACE"
  fi

  ip addr flush dev "$HIKVISION_IFACE" scope global || true
  ip addr add "$HIKVISION_HOST_IP" dev "$HIKVISION_IFACE"
  ip link set dev "$HIKVISION_IFACE" up
  ip route replace "$HIKVISION_CAMERA_IP/32" dev "$HIKVISION_IFACE" src "$HIKVISION_HOST_ADDR"
fi

sleep 1
print_state

if ! iface_has_carrier "$HIKVISION_IFACE"; then
  echo "[start_hikvision_network] $HIKVISION_IFACE has no physical link: carrier=0" >&2
  echo "[start_hikvision_network] check Hikvision camera power/PoE, Ethernet cable, and USB-RJ45 adapter link lights" >&2
  exit 1
fi
