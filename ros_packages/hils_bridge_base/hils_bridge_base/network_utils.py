"""
Network utilities for HILS bridge nodes.

Provides helper functions for:
  - Detecting IP addresses and netmasks on network interfaces
  - Binding UDP sockets to specific interfaces (e.g. USB-Ethernet adapters)
  - Subnet validation between emulator and host IPs

Extracted from livox_emulator_node.py for shared use across all
UDP-based emulator nodes (Livox, Velodyne, Ouster, etc.).
"""

import fcntl
import os
import socket
import struct
from typing import Optional, Tuple

# ioctl constants for Linux
SIOCGIFADDR = 0x8915
SIOCGIFNETMASK = 0x891B


def get_interface_ip_and_netmask(iface: str) -> Tuple[str, Optional[str]]:
    """Get IPv4 address and netmask of a network interface using ioctl.

    Args:
        iface: Network interface name (e.g. "eth1")

    Returns:
        (ip, netmask) tuple. netmask may be None if unavailable.

    Raises:
        RuntimeError: If the interface has no IP address.
    """
    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Get IP
    req = bytearray(40)
    req[:len(iface)] = iface.encode()
    try:
        fcntl.ioctl(ctrl_sock.fileno(), SIOCGIFADDR, req)
    except OSError:
        ctrl_sock.close()
        raise RuntimeError(
            f'Cannot get IP of {iface}. '
            f'Check the interface exists and has an IP assigned.')
    ip = socket.inet_ntoa(req[20:24])

    # Get netmask
    req = bytearray(40)
    req[:len(iface)] = iface.encode()
    try:
        fcntl.ioctl(ctrl_sock.fileno(), SIOCGIFNETMASK, req)
        netmask = socket.inet_ntoa(req[20:24])
    except OSError:
        netmask = None

    ctrl_sock.close()
    return ip, netmask


def find_netmask_for_ip(target_ip: str) -> Optional[str]:
    """Find the netmask for a given IP by scanning all network interfaces.

    Args:
        target_ip: IP address to look up.

    Returns:
        Netmask string, or None if the IP is not found on any interface.
    """
    try:
        ifaces = os.listdir('/sys/class/net/')
    except OSError:
        return None

    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for iface in ifaces:
        req = bytearray(40)
        req[:len(iface)] = iface.encode()
        try:
            fcntl.ioctl(ctrl_sock.fileno(), SIOCGIFADDR, req)
            ip = socket.inet_ntoa(req[20:24])
            if ip == target_ip:
                req2 = bytearray(40)
                req2[:len(iface)] = iface.encode()
                fcntl.ioctl(ctrl_sock.fileno(), SIOCGIFNETMASK, req2)
                netmask = socket.inet_ntoa(req2[20:24])
                ctrl_sock.close()
                return netmask
        except OSError:
            continue
    ctrl_sock.close()
    return None


def find_interface_for_ip(target_ip: str) -> Optional[str]:
    """Find the network interface name that has a given IP assigned.

    Args:
        target_ip: IP address to look up.

    Returns:
        Interface name (e.g. "eth1"), or None if not found.
    """
    try:
        ifaces = os.listdir('/sys/class/net/')
    except OSError:
        return None

    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for iface in ifaces:
        req = bytearray(40)
        req[:len(iface)] = iface.encode()
        try:
            fcntl.ioctl(ctrl_sock.fileno(), SIOCGIFADDR, req)
            ip = socket.inet_ntoa(req[20:24])
            if ip == target_ip:
                ctrl_sock.close()
                return iface
        except OSError:
            continue
    ctrl_sock.close()
    return None


def validate_subnet(ip_a: str, ip_b: str, netmask: str) -> bool:
    """Check that two IPs are on the same subnet.

    Args:
        ip_a: First IP address.
        ip_b: Second IP address.
        netmask: Subnet mask.

    Returns:
        True if both IPs are on the same subnet.
    """
    def ip_to_int(ip):
        return struct.unpack('!I', socket.inet_aton(ip))[0]

    mask = ip_to_int(netmask)
    return (ip_to_int(ip_a) & mask) == (ip_to_int(ip_b) & mask)


def verify_ip_available(ip: str) -> bool:
    """Check if an IP address can be bound (i.e., is assigned to a local interface).

    Args:
        ip: IP address to test.

    Returns:
        True if the IP can be bound.
    """
    test_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        test_sock.bind((ip, 0))
        test_sock.close()
        return True
    except OSError:
        return False


def create_udp_socket(bind_ip: str, port: int, *,
                      network_interface: str = '',
                      reuse: bool = False) -> socket.socket:
    """Create and bind a non-blocking UDP socket.

    Args:
        bind_ip: IP address to bind. Use '' for INADDR_ANY.
        port: Port number to bind.
        network_interface: If set, bind to this interface via SO_BINDTODEVICE.
        reuse: If True, set SO_REUSEADDR.

    Returns:
        Configured socket.
    """
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if reuse:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if network_interface:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                        network_interface.encode() + b'\0')
    sock.setblocking(False)
    sock.bind((bind_ip, port))
    return sock
