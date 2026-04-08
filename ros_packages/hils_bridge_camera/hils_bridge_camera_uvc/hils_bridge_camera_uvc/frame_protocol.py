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

# Message types (first byte of reverse-channel payloads)
MSG_TYPE_RESOLUTION_CMD = 0x01


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


def parse_resolution_cmd(payload: bytes):
    """Parse resolution command from payload.

    Returns (width, height, frame_index) or None if not a resolution command.
    """
    if len(payload) < 6 or payload[0] != MSG_TYPE_RESOLUTION_CMD:
        return None
    width = int.from_bytes(payload[1:3], 'little')
    height = int.from_bytes(payload[3:5], 'little')
    frame_index = payload[5]
    return (width, height, frame_index)


class FrameProtocolReceiver:
    """State machine to parse HILS framed messages from a byte stream.

    Usage:
        receiver = FrameProtocolReceiver()
        for payload in receiver.feed(data):
            # process complete payload
    """

    _WAIT_SYNC0 = 0
    _WAIT_SYNC1 = 1
    _READ_LENGTH = 2
    _READ_PAYLOAD = 3
    _READ_CHECKSUM = 4

    def __init__(self, max_payload=64):
        self._max_payload = max_payload
        self._reset()

    def _reset(self):
        self._state = self._WAIT_SYNC0
        self._length_buf = bytearray()
        self._payload_length = 0
        self._payload = bytearray()
        self._checksum = 0

    def feed(self, data: bytes):
        """Feed raw bytes, yield complete payloads."""
        for byte in data:
            if self._state == self._WAIT_SYNC0:
                if byte == SYNC_0:
                    self._state = self._WAIT_SYNC1
            elif self._state == self._WAIT_SYNC1:
                if byte == SYNC_1:
                    self._state = self._READ_LENGTH
                    self._length_buf = bytearray()
                elif byte == SYNC_0:
                    pass  # stay
                else:
                    self._reset()
            elif self._state == self._READ_LENGTH:
                self._length_buf.append(byte)
                if len(self._length_buf) == 4:
                    self._payload_length = int.from_bytes(self._length_buf, 'little')
                    if self._payload_length == 0 or self._payload_length > self._max_payload:
                        self._reset()
                    else:
                        self._payload = bytearray()
                        self._checksum = 0
                        self._state = self._READ_PAYLOAD
            elif self._state == self._READ_PAYLOAD:
                self._payload.append(byte)
                self._checksum ^= byte
                if len(self._payload) >= self._payload_length:
                    self._state = self._READ_CHECKSUM
            elif self._state == self._READ_CHECKSUM:
                if byte == (self._checksum & 0xFF):
                    yield bytes(self._payload)
                self._reset()
