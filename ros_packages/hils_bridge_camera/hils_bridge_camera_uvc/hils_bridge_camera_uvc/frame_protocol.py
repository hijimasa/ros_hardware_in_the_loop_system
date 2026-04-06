"""
HILS Frame Protocol - Python implementation.

Matches the C protocol defined in firmware/common/include/hils_frame_protocol.h.

Frame format:
    [0xAA][0x55][4-byte payload length, little-endian][payload][1-byte XOR checksum]
"""

import struct

SYNC_0 = 0xAA
SYNC_1 = 0x55
HEADER_SIZE = 6
MAX_PAYLOAD = 65536
OVERHEAD = 7  # header + checksum


def compute_checksum(data: bytes) -> int:
    """Compute XOR checksum over data bytes."""
    result = 0
    for b in data:
        result ^= b
    return result & 0xFF


def build_frame(payload: bytes) -> bytes:
    """Wrap payload in HILS frame protocol (header + checksum)."""
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"Payload size {len(payload)} exceeds max {MAX_PAYLOAD}")
    header = struct.pack('<BBL', SYNC_0, SYNC_1, len(payload))
    checksum = compute_checksum(payload)
    return header + payload + bytes([checksum])
