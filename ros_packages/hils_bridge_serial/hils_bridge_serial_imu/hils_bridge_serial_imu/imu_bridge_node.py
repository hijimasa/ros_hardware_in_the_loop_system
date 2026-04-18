#!/usr/bin/env python3
"""
HILS IMU Serial Bridge Node (Witmotion WT901 Protocol)

Subscribes to a ROS sensor_msgs/Imu topic from a simulator and writes
Witmotion WT901-compatible binary packets to a serial port (FT234X
USB-serial adapter with cross-connection to the robot PC).

Each IMU message produces three 11-byte packets (33 bytes total):
  0x51 - Acceleration
  0x52 - Angular Velocity
  0x53 - Angle (Euler)

Data flow:
    Simulation (Imu) -> this node -> WT901 binary serial -> FT234X -> Robot PC
"""

import math
import struct

import rclpy
from sensor_msgs.msg import Imu

from hils_bridge_base.serial_bridge_base import SerialBridgeBase

# Witmotion WT901 protocol constants
_HEADER = 0x55
_TYPE_ACCEL = 0x51
_TYPE_GYRO = 0x52
_TYPE_ANGLE = 0x53

# Conversion constants
_GRAVITY = 9.8
_INT16_MIN = -32768
_INT16_MAX = 32767

# Default temperature raw value: 25.0 °C * 100 = 2500
_TEMP_RAW = 2500


def _clamp_int16(value: int) -> int:
    """Clamp a value to the signed 16-bit integer range [-32768, 32767].

    Args:
        value: Integer value to clamp.

    Returns:
        Clamped integer value.
    """
    if value < _INT16_MIN:
        return _INT16_MIN
    if value > _INT16_MAX:
        return _INT16_MAX
    return value


def _build_packet(packet_type: int, d0: int, d1: int, d2: int, d3: int) -> bytes:
    """Build an 11-byte Witmotion WT901 binary packet.

    Packet format: [0x55][Type][D0L][D0H][D1L][D1H][D2L][D2H][D3L][D3H][SUM]
    where D0-D3 are signed 16-bit little-endian values.

    Args:
        packet_type: Packet type byte (0x51, 0x52, or 0x53).
        d0: First data value (int16).
        d1: Second data value (int16).
        d2: Third data value (int16).
        d3: Fourth data value (int16).

    Returns:
        11-byte packet as bytes.
    """
    data = struct.pack('<BBhhhh', _HEADER, packet_type, d0, d1, d2, d3)
    checksum = sum(data) & 0xFF
    return data + struct.pack('B', checksum)


def _quaternion_to_euler(x: float, y: float, z: float, w: float) -> tuple:
    """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees.

    Uses the standard aerospace rotation sequence (ZYX).

    Args:
        x: Quaternion x component.
        y: Quaternion y component.
        z: Quaternion z component.
        w: Quaternion w component.

    Returns:
        Tuple of (roll, pitch, yaw) in degrees.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


class ImuBridgeNode(SerialBridgeBase):
    """ROS2 node that converts Imu messages to Witmotion WT901 binary serial output."""

    def __init__(self):
        super().__init__(
            node_name='hils_imu_bridge',
            default_baudrate=115200,
        )

        # Additional parameters
        self.declare_parameter('imu_topic', '/imu/data')

        # Subscribe to Imu
        imu_topic = self.get_parameter('imu_topic').value
        self.create_subscription(Imu, imu_topic, self._imu_callback, 1)

        self.get_logger().info(
            f'IMU WT901 Bridge started: imu_topic={imu_topic}, '
            f'baudrate={self.get_parameter("baudrate").value}, '
            f'max_hz={self.get_parameter("max_hz").value}')

    def _imu_callback(self, msg: Imu):
        """Convert Imu message to WT901 binary packets and write to serial.

        Generates three packets (acceleration, angular velocity, angle) and
        writes them as a single 33-byte block.
        """
        if not self.check_rate_limit():
            return

        # --- 0x51 Acceleration ---
        # Physical value = raw / 32768 * 16 * g
        # => raw = value / g / 16 * 32768
        ax_raw = _clamp_int16(int(msg.linear_acceleration.x / _GRAVITY / 16.0 * 32768.0))
        ay_raw = _clamp_int16(int(msg.linear_acceleration.y / _GRAVITY / 16.0 * 32768.0))
        az_raw = _clamp_int16(int(msg.linear_acceleration.z / _GRAVITY / 16.0 * 32768.0))
        pkt_accel = _build_packet(_TYPE_ACCEL, ax_raw, ay_raw, az_raw, _TEMP_RAW)

        # --- 0x52 Angular Velocity ---
        # Physical value = raw / 32768 * 2000 (°/s)
        # => raw = value_deg_s / 2000 * 32768
        wx_deg = math.degrees(msg.angular_velocity.x)
        wy_deg = math.degrees(msg.angular_velocity.y)
        wz_deg = math.degrees(msg.angular_velocity.z)
        wx_raw = _clamp_int16(int(wx_deg / 2000.0 * 32768.0))
        wy_raw = _clamp_int16(int(wy_deg / 2000.0 * 32768.0))
        wz_raw = _clamp_int16(int(wz_deg / 2000.0 * 32768.0))
        pkt_gyro = _build_packet(_TYPE_GYRO, wx_raw, wy_raw, wz_raw, _TEMP_RAW)

        # --- 0x53 Angle (Euler) ---
        # Physical value = raw / 32768 * 180 (°)
        # => raw = deg / 180 * 32768
        roll, pitch, yaw = _quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        roll_raw = _clamp_int16(int(roll / 180.0 * 32768.0))
        pitch_raw = _clamp_int16(int(pitch / 180.0 * 32768.0))
        yaw_raw = _clamp_int16(int(yaw / 180.0 * 32768.0))
        pkt_angle = _build_packet(_TYPE_ANGLE, roll_raw, pitch_raw, yaw_raw, 0x0000)

        # Write all three packets as one block (33 bytes)
        self.serial_write(pkt_accel + pkt_gyro + pkt_angle)


def main(args=None):
    rclpy.init(args=args)
    node = ImuBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
