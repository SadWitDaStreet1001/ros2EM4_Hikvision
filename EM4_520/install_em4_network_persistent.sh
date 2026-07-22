#!/usr/bin/env bash
set -euo pipefail

LIDAR_IFACE="lidar0"
LIDAR_MAC="20:7b:d5:1a:07:0a"
LIDAR_HOST_IP="192.168.1.102/24"

HIKVISION_IFACE="hikvision"
HIKVISION_MAC="6c:1f:f7:df:9f:fb"
HIKVISION_HOST_ADDR="192.168.1.103"
HIKVISION_HOST_IP="${HIKVISION_HOST_ADDR}/32"
HIKVISION_CAMERA_IP="192.168.1.64"

if [[ ${EUID} -ne 0 ]]; then
  if command -v sudo >/dev/null 2>&1; then
    exec sudo -E "$0" "$@"
  fi
  echo "[install_em4_network_persistent] please run as root" >&2
  exit 1
fi

if command -v systemd-detect-virt >/dev/null 2>&1; then
  virt="$(systemd-detect-virt --container 2>/dev/null || true)"
  if [[ -n "${virt}" ]]; then
    echo "[install_em4_network_persistent] current shell is inside a container: ${virt}" >&2
    echo "[install_em4_network_persistent] run this script on the Linux host, not inside Docker" >&2
    exit 2
  fi
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

write_systemd_link_files() {
  mkdir -p /etc/systemd/network

  cat >/etc/systemd/network/10-em4-lidar.link <<EOF
[Match]
MACAddress=${LIDAR_MAC}

[Link]
Name=${LIDAR_IFACE}
EOF

  cat >/etc/systemd/network/10-em4-hikvision.link <<EOF
[Match]
MACAddress=${HIKVISION_MAC}

[Link]
Name=${HIKVISION_IFACE}
EOF
}

write_networkd_files() {
  mkdir -p /etc/systemd/network

  cat >/etc/systemd/network/20-em4-lidar.network <<EOF
[Match]
Name=${LIDAR_IFACE}

[Network]
DHCP=no
LinkLocalAddressing=no
Address=${LIDAR_HOST_IP}
EOF

  cat >/etc/systemd/network/20-em4-hikvision.network <<EOF
[Match]
Name=${HIKVISION_IFACE}

[Network]
DHCP=no
LinkLocalAddressing=no
Address=${HIKVISION_HOST_IP}

[Route]
Destination=${HIKVISION_CAMERA_IP}/32
PreferredSource=${HIKVISION_HOST_ADDR}
EOF
}

configure_networkmanager() {
  command -v nmcli >/dev/null 2>&1 || return 1
  systemctl is-active --quiet NetworkManager 2>/dev/null || return 1

  nmcli connection delete em4-lidar >/dev/null 2>&1 || true
  nmcli connection add type ethernet con-name em4-lidar ifname "${LIDAR_IFACE}" >/dev/null
  nmcli connection modify em4-lidar \
    802-3-ethernet.mac-address "${LIDAR_MAC}" \
    connection.autoconnect yes \
    ipv4.method manual \
    ipv4.addresses "${LIDAR_HOST_IP}" \
    ipv4.never-default yes \
    ipv6.method ignore

  nmcli connection delete em4-hikvision >/dev/null 2>&1 || true
  nmcli connection add type ethernet con-name em4-hikvision ifname "${HIKVISION_IFACE}" >/dev/null
  nmcli connection modify em4-hikvision \
    802-3-ethernet.mac-address "${HIKVISION_MAC}" \
    connection.autoconnect yes \
    ipv4.method manual \
    ipv4.addresses "${HIKVISION_HOST_IP}" \
    ipv4.routes "${HIKVISION_CAMERA_IP}/32 0.0.0.0" \
    ipv4.never-default yes \
    ipv6.method ignore

  nmcli connection reload >/dev/null || true
  nmcli connection up em4-lidar >/dev/null 2>&1 || true
  nmcli connection up em4-hikvision >/dev/null 2>&1 || true
}

configure_current_runtime() {
  command -v ip >/dev/null 2>&1 || return 0

  local iface
  iface="${LIDAR_IFACE}"
  if ! ip link show "${iface}" >/dev/null 2>&1; then
    iface="$(find_iface_by_mac "${LIDAR_MAC}")" || iface=""
  fi
  if [[ -n "${iface}" ]]; then
    ip link set dev "${iface}" down
    if [[ "${iface}" != "${LIDAR_IFACE}" ]]; then
      ip link set dev "${iface}" name "${LIDAR_IFACE}"
    fi
    ip addr flush dev "${LIDAR_IFACE}" scope global || true
    ip addr add "${LIDAR_HOST_IP}" dev "${LIDAR_IFACE}"
    ip link set dev "${LIDAR_IFACE}" up
  fi

  iface="${HIKVISION_IFACE}"
  if ! ip link show "${iface}" >/dev/null 2>&1; then
    iface="$(find_iface_by_mac "${HIKVISION_MAC}")" || iface=""
  fi
  if [[ -n "${iface}" ]]; then
    ip link set dev "${iface}" down
    if [[ "${iface}" != "${HIKVISION_IFACE}" ]]; then
      ip link set dev "${iface}" name "${HIKVISION_IFACE}"
    fi
    ip addr flush dev "${HIKVISION_IFACE}" scope global || true
    ip addr add "${HIKVISION_HOST_IP}" dev "${HIKVISION_IFACE}"
    ip link set dev "${HIKVISION_IFACE}" up
    ip route replace "${HIKVISION_CAMERA_IP}/32" dev "${HIKVISION_IFACE}" src "${HIKVISION_HOST_ADDR}"
  fi
}

echo "[install_em4_network_persistent] writing persistent interface names"
write_systemd_link_files

if configure_networkmanager; then
  echo "[install_em4_network_persistent] NetworkManager profiles installed"
else
  echo "[install_em4_network_persistent] NetworkManager is not active; writing systemd-networkd files"
  write_networkd_files
  if [[ "$(ps -p 1 -o comm= 2>/dev/null || true)" == "systemd" ]]; then
    systemctl enable systemd-networkd >/dev/null 2>&1 || true
    systemctl restart systemd-networkd >/dev/null 2>&1 || true
  fi
fi

if command -v udevadm >/dev/null 2>&1; then
  udevadm control --reload-rules >/dev/null 2>&1 || true
fi

configure_current_runtime

sleep 1
echo "[install_em4_network_persistent] current state:"
ip -br addr show "${LIDAR_IFACE}" 2>/dev/null || true
ip -br addr show "${HIKVISION_IFACE}" 2>/dev/null || true
ip route get "${HIKVISION_CAMERA_IP}" 2>/dev/null || true

echo "[install_em4_network_persistent] done"
