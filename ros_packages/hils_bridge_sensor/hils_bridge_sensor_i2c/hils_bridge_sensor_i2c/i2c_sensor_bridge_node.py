#!/usr/bin/env python3
"""
HILS I2C IMU Sensor Bridge Node

Subscribes to a ROS Imu topic from a simulator, converts accelerometer
and gyroscope data to MPU-6050 raw register values, and sends them to
an RP2040 via USB CDC using the HILS framing protocol.

Data flow:
    /imu/data -> SI-to-raw conversion -> frame protocol -> serial -> RP2040
    RP2040 -> I2C slave registers (MPU-6050 emulation)

The RP2040 firmware maintains a register map that an external I2C master
can read, making the system appear as a real MPU-6050 IMU sensor.

MPU-6050 default sensitivity:
    Accel: +/-2g  -> 16384 LSB/g
    Gyro:  +/-250 deg/s -> 131 LSB/(deg/s)
    Temp:  raw = (degC - 36.53) * 340
"""

import struct

import rclpy
from sensor_msgs.msg import Imu

from hils_bridge_base.serial_bridge_base import SerialBridgeBase
from hils_bridge_base import frame_protocol

# Message type for I2C sensor data (must match firmware)
MSG_TYPE_I2C_SENSOR = 0x30

# MPU-6050 sensitivity constants (default full-scale ranges)
ACCEL_LSB_PER_G = 16384.0          # +/-2g range: 16384 LSB/g
GYRO_LSB_PER_DPS = 131.0           # +/-250 deg/s range: 131 LSB/(deg/s)
GRAVITY_MS2 = 9.80665              # m/s^2 per g
RAD_TO_DEG = 57.29577951308232     # 180/pi

# Default temperature raw value (25 deg C)
# MPU-6050: Temperature = (raw / 340) + 36.53
# raw = (25 - 36.53) * 340 = -3920.2 -> -3920
DEFAULT_TEMP_RAW = -3920


def clamp_int16(value: int) -> int:
    """Clamp an integer to the int16 range [-32768, 32767]."""
    if value > 32767:
        return 32767
    if value < -32768:
        return -32768
    return value


class I2cSensorBridgeNode(SerialBridgeBase):
    """ROS2 node that converts Imu messages to MPU-6050 register data."""

    def __init__(self):
        super().__init__(
            node_name='hils_i2c_sensor_bridge',
            default_baudrate=115200,
        )

        # Declare parameters
        self.declare_parameter('imu_topic', '/imu/data')

        # Subscribe to Imu
        topic = self.get_parameter('imu_topic').value
        self.create_subscription(Imu, topic, self._imu_callback, 10)

        self.get_logger().info(
            f'I2C IMU Sensor Bridge started: topic={topic}, '
            f'accel_scale={ACCEL_LSB_PER_G} LSB/g, '
            f'gyro_scale={GYRO_LSB_PER_DPS} LSB/(deg/s)')

    def _accel_to_raw(self, accel_ms2: float) -> int:
        """Convert acceleration from m/s^2 to MPU-6050 raw int16.

        MPU-6050 at +/-2g range: 16384 LSB per g (9.80665 m/s^2).

        Args:
            accel_ms2: Acceleration in m/s^2.

        Returns:
            Raw int16 value for MPU-6050 register (big-endian convention).
        """
        raw = int(accel_ms2 / GRAVITY_MS2 * ACCEL_LSB_PER_G + 0.5)
        return clamp_int16(raw)

    def _gyro_to_raw(self, gyro_rads: float) -> int:
        """Convert angular velocity from rad/s to MPU-6050 raw int16.

        MPU-6050 at +/-250 deg/s range: 131 LSB per deg/s.

        Args:
            gyro_rads: Angular velocity in rad/s.

        Returns:
            Raw int16 value for MPU-6050 register (big-endian convention).
        """
        gyro_dps = gyro_rads * RAD_TO_DEG
        raw = int(gyro_dps * GYRO_LSB_PER_DPS + 0.5)
        return clamp_int16(raw)

    def _imu_callback(self, msg: Imu):
        """Convert Imu message to MPU-6050 raw data and send via serial."""
        if not self.check_rate_limit():
            return

        # Convert accelerometer data (m/s^2 -> raw int16)
        accel_x = self._accel_to_raw(msg.linear_acceleration.x)
        accel_y = self._accel_to_raw(msg.linear_acceleration.y)
        accel_z = self._accel_to_raw(msg.linear_acceleration.z)

        # Convert gyroscope data (rad/s -> raw int16)
        gyro_x = self._gyro_to_raw(msg.angular_velocity.x)
        gyro_y = self._gyro_to_raw(msg.angular_velocity.y)
        gyro_z = self._gyro_to_raw(msg.angular_velocity.z)

        # Temperature: use default 25 deg C
        temp = DEFAULT_TEMP_RAW

        # Build binary payload
        # msg_type (1 byte) + 7 x int16 (14 bytes) = 15 bytes total
        # int16 values are sent as little-endian over CDC (firmware converts
        # to big-endian for MPU-6050 register map)
        payload = struct.pack('<Bhhhhhhh',
                              MSG_TYPE_I2C_SENSOR,
                              accel_x, accel_y, accel_z,
                              gyro_x, gyro_y, gyro_z,
                              temp)

        # Wrap in HILS frame protocol and send
        frame_data = frame_protocol.build_frame(payload)
        self.serial_write(frame_data)


def main(args=None):
    rclpy.init(args=args)
    node = I2cSensorBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
