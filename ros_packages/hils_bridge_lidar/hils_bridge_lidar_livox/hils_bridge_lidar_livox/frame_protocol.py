"""
HILS Frame Protocol - LiDAR extension.

Builds on the base frame protocol (AA 55 framing) with LiDAR-specific
message types for Livox Mid360 emulation.

Message types:
    0x10: Point cloud batch (forward: ROS node -> firmware)
    0x11: IMU data (forward)
    0x12: Configuration command (forward)
    0x18: Status report (reverse: firmware -> ROS node)
"""

import struct

SYNC_0 = 0xAA
SYNC_1 = 0x55
MAX_PAYLOAD = 65536

# Message types
MSG_TYPE_LIDAR_POINTS = 0x10
MSG_TYPE_LIDAR_IMU = 0x11
MSG_TYPE_LIDAR_CONFIG = 0x12
MSG_TYPE_LIDAR_STATUS = 0x18

# Config sub-commands
CFG_SET_HOST_IP = 0x01
CFG_SET_LIDAR_IP = 0x02
CFG_SET_SN = 0x03
CFG_START = 0x04
CFG_STOP = 0x05

# Point data types
DATA_TYPE_CARTESIAN_HIGH = 1  # 14 bytes/point: x,y,z(int32 mm) + reflectivity + tag
DATA_TYPE_CARTESIAN_LOW = 2   # 8 bytes/point: x,y,z(int16 cm) + reflectivity + tag

POINT_SIZE_HIGH = 14  # int32 x,y,z (12) + reflectivity (1) + tag (1)
POINT_SIZE_LOW = 8    # int16 x,y,z (6) + reflectivity (1) + tag (1)


def compute_checksum(data: bytes) -> int:
    result = 0
    for b in data:
        result ^= b
    return result & 0xFF


def build_frame(payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"Payload size {len(payload)} exceeds max {MAX_PAYLOAD}")
    header = struct.pack('<BBL', SYNC_0, SYNC_1, len(payload))
    checksum = compute_checksum(payload)
    return header + payload + bytes([checksum])


def build_points_frame(data_type: int, dot_num: int, timestamp_ns: int,
                       point_data: bytes) -> bytes:
    """Build a point cloud batch frame.

    Args:
        data_type: 1=CartesianHigh, 2=CartesianLow
        dot_num: Number of points
        timestamp_ns: 64-bit timestamp in nanoseconds
        point_data: Raw point data bytes
    """
    ts_lo = timestamp_ns & 0xFFFFFFFF
    ts_hi = (timestamp_ns >> 32) & 0xFFFFFFFF
    header = struct.pack('<BBHII',
                         MSG_TYPE_LIDAR_POINTS,
                         data_type,
                         dot_num,
                         ts_lo, ts_hi)
    payload = header + point_data
    return build_frame(payload)


def build_imu_frame(gyro_x: float, gyro_y: float, gyro_z: float,
                    acc_x: float, acc_y: float, acc_z: float) -> bytes:
    """Build an IMU data frame."""
    payload = struct.pack('<Bffffff',
                          MSG_TYPE_LIDAR_IMU,
                          gyro_x, gyro_y, gyro_z,
                          acc_x, acc_y, acc_z)
    return build_frame(payload)


def build_config_frame(sub_cmd: int, data: bytes = b'') -> bytes:
    """Build a configuration command frame."""
    payload = struct.pack('<BB', MSG_TYPE_LIDAR_CONFIG, sub_cmd) + data
    return build_frame(payload)


def build_set_ip_frame(sub_cmd: int, ip_str: str) -> bytes:
    """Build a set IP config frame (host or lidar IP)."""
    parts = [int(x) for x in ip_str.split('.')]
    ip_bytes = bytes(parts)
    return build_config_frame(sub_cmd, ip_bytes)


def build_set_sn_frame(serial_number: str) -> bytes:
    """Build a set serial number config frame."""
    sn_bytes = serial_number.encode('ascii')[:16].ljust(16, b'\x00')
    return build_config_frame(CFG_SET_SN, sn_bytes)


def parse_status(payload: bytes):
    """Parse a status report from firmware.

    Returns (state, points_sent, udp_errors) or None.
    """
    if len(payload) < 10 or payload[0] != MSG_TYPE_LIDAR_STATUS:
        return None
    state = payload[1]
    points_sent = struct.unpack_from('<I', payload, 2)[0]
    udp_errors = struct.unpack_from('<I', payload, 6)[0]
    return (state, points_sent, udp_errors)


class FrameProtocolReceiver:
    """State machine to parse HILS framed messages from a byte stream."""

    _WAIT_SYNC0 = 0
    _WAIT_SYNC1 = 1
    _READ_LENGTH = 2
    _READ_PAYLOAD = 3
    _READ_CHECKSUM = 4

    def __init__(self, max_payload=256):
        self._max_payload = max_payload
        self._reset()

    def _reset(self):
        self._state = self._WAIT_SYNC0
        self._length_buf = bytearray()
        self._payload_length = 0
        self._payload = bytearray()
        self._checksum = 0

    def feed(self, data: bytes):
        for byte in data:
            if self._state == self._WAIT_SYNC0:
                if byte == SYNC_0:
                    self._state = self._WAIT_SYNC1
            elif self._state == self._WAIT_SYNC1:
                if byte == SYNC_1:
                    self._state = self._READ_LENGTH
                    self._length_buf = bytearray()
                elif byte == SYNC_0:
                    pass
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
