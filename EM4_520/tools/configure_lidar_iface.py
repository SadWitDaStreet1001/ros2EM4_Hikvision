#!/usr/bin/env python3
"""Configure a network interface IPv4 address without relying on iproute2."""

import ipaddress
import socket
import struct
import sys
from typing import Optional, Tuple

import psutil

import fcntl


SIOCGIFFLAGS = 0x8913
SIOCSIFFLAGS = 0x8914
SIOCSIFADDR = 0x8916
SIOCSIFNETMASK = 0x891c
IFF_UP = 0x1
IFNAMSIZ = 16


def pack_ifreq(name: str, payload: bytes) -> bytes:
    ifname = name.encode("ascii")
    if len(ifname) >= IFNAMSIZ:
        raise ValueError(f"Interface name too long: {name}")
    return struct.pack(f"{IFNAMSIZ}s", ifname) + payload


def sockaddr_ipv4(ipv4: str) -> bytes:
    return struct.pack("H2s4s8s", socket.AF_INET, b"\x00" * 2, socket.inet_aton(ipv4), b"\x00" * 8)


def current_ipv4(name: str) -> Optional[Tuple[str, str]]:
    for addr in psutil.net_if_addrs().get(name, []):
        if addr.family == socket.AF_INET:
            return addr.address, addr.netmask
    return None


def ensure_iface_up(sock: socket.socket, name: str) -> None:
    ifreq = pack_ifreq(name, b"\x00" * 24)
    res = fcntl.ioctl(sock.fileno(), SIOCGIFFLAGS, ifreq)
    flags = struct.unpack_from("H", res, IFNAMSIZ)[0]
    if flags & IFF_UP:
        return
    updated = bytearray(ifreq)
    struct.pack_into("H", updated, IFNAMSIZ, flags | IFF_UP)
    fcntl.ioctl(sock.fileno(), SIOCSIFFLAGS, updated)


def configure_ipv4(name: str, cidr: str) -> None:
    iface = psutil.net_if_addrs().get(name)
    if iface is None:
        raise RuntimeError(f"Interface not found: {name}")

    network = ipaddress.IPv4Interface(cidr)
    want_ip = str(network.ip)
    want_mask = str(network.netmask)
    have_addr = current_ipv4(name)
    if have_addr == (want_ip, want_mask):
        return

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        ensure_iface_up(sock, name)
        fcntl.ioctl(sock.fileno(), SIOCSIFADDR, pack_ifreq(name, sockaddr_ipv4(want_ip)))
        fcntl.ioctl(sock.fileno(), SIOCSIFNETMASK, pack_ifreq(name, sockaddr_ipv4(want_mask)))


def main() -> int:
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <iface> <ipv4/cidr>", file=sys.stderr)
        return 2

    iface = sys.argv[1]
    cidr = sys.argv[2]
    try:
        configure_ipv4(iface, cidr)
    except Exception as exc:
        print(f"[configure_lidar_iface] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
