#!/usr/bin/env python3
"""
HILS PWM Servo + Encoder Bridge Node

Subscribes to a ROS JointState topic from a simulator, converts joint
positions to servo pulse widths and encoder counts, and sends them to
an RP2040 via USB CDC using the HILS framing protocol.

Data flow:
    /joint_states -> angle-to-pulse conversion -> frame protocol -> serial -> RP2040
    RP2040 -> PIO PWM servo signals + quadrature encoder outputs

Each joint in the JointState message is mapped to a servo channel and
optionally an encoder channel based on the 'joint_names' parameter.
"""

import struct
import math

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from sensor_msgs.msg import JointState

from hils_bridge_base.serial_bridge_base import SerialBridgeBase
from hils_bridge_base import frame_protocol

# Message type for servo commands (must match firmware)
MSG_TYPE_SERVO_CMD = 0x20


class PwmBridgeNode(SerialBridgeBase):
    """ROS2 node that converts JointState messages to servo/encoder commands."""

    def __init__(self):
        super().__init__(
            node_name='hils_pwm_bridge',
            default_baudrate=115200,
        )

        # Declare parameters
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('servo_channels', 4,
            ParameterDescriptor(description='Number of servo channels (1-4)'))
        self.declare_parameter('encoder_cpr', 1000,
            ParameterDescriptor(description='Encoder counts per revolution'))
        self.declare_parameter('servo_min_angle', -math.pi / 2,
            ParameterDescriptor(
                description='Minimum servo angle in radians',
                floating_point_range=[FloatingPointRange(
                    from_value=-math.pi, to_value=0.0, step=0.0)]))
        self.declare_parameter('servo_max_angle', math.pi / 2,
            ParameterDescriptor(
                description='Maximum servo angle in radians',
                floating_point_range=[FloatingPointRange(
                    from_value=0.0, to_value=math.pi, step=0.0)]))
        self.declare_parameter('servo_min_pulse_us', 500,
            ParameterDescriptor(description='Minimum servo pulse width (us)'))
        self.declare_parameter('servo_max_pulse_us', 2500,
            ParameterDescriptor(description='Maximum servo pulse width (us)'))
        self.declare_parameter('joint_names', [''],
            ParameterDescriptor(
                description='Ordered list of joint names to map to channels. '
                            'Empty string means use positional index.'))

        # Subscribe to JointState
        topic = self.get_parameter('joint_state_topic').value
        self.create_subscription(
            JointState, topic, self._joint_state_callback, 1)

        self.get_logger().info(
            f'PWM Servo Bridge started: topic={topic}, '
            f'channels={self.get_parameter("servo_channels").value}, '
            f'encoder_cpr={self.get_parameter("encoder_cpr").value}, '
            f'pulse=[{self.get_parameter("servo_min_pulse_us").value}, '
            f'{self.get_parameter("servo_max_pulse_us").value}] us, '
            f'angle=[{self.get_parameter("servo_min_angle").value:.3f}, '
            f'{self.get_parameter("servo_max_angle").value:.3f}] rad')

    def _angle_to_pulse_us(self, angle_rad: float) -> int:
        """Convert joint angle (radians) to servo pulse width (microseconds).

        Linearly maps [servo_min_angle, servo_max_angle] to
        [servo_min_pulse_us, servo_max_pulse_us].

        Args:
            angle_rad: Joint angle in radians.

        Returns:
            Pulse width in microseconds, clamped to [500, 2500].
        """
        min_angle = self.get_parameter('servo_min_angle').value
        max_angle = self.get_parameter('servo_max_angle').value
        min_pulse = self.get_parameter('servo_min_pulse_us').value
        max_pulse = self.get_parameter('servo_max_pulse_us').value

        # Clamp input angle
        angle_rad = max(min_angle, min(max_angle, angle_rad))

        # Linear interpolation
        t = (angle_rad - min_angle) / (max_angle - min_angle)
        pulse = int(min_pulse + t * (max_pulse - min_pulse) + 0.5)

        return max(500, min(2500, pulse))

    def _angle_to_encoder_count(self, angle_rad: float) -> int:
        """Convert joint angle (radians) to encoder count.

        Maps angle in radians to encoder counts using counts-per-revolution.
        count = angle * (cpr / (2 * pi))

        Args:
            angle_rad: Joint angle in radians.

        Returns:
            Encoder count (signed integer).
        """
        cpr = self.get_parameter('encoder_cpr').value
        count = int(angle_rad * cpr / (2.0 * math.pi) + 0.5)
        return count

    def _find_joint_index(self, msg: JointState, joint_name: str) -> int:
        """Find the index of a joint name in the JointState message.

        Args:
            msg: JointState message.
            joint_name: Name to search for.

        Returns:
            Index in msg.name, or -1 if not found.
        """
        try:
            return list(msg.name).index(joint_name)
        except ValueError:
            return -1

    def _joint_state_callback(self, msg: JointState):
        """Convert JointState to servo commands and send via serial."""
        if not self.check_rate_limit():
            return

        num_channels = self.get_parameter('servo_channels').value
        joint_names = self.get_parameter('joint_names').value

        # Build channel data
        channels = []

        for ch in range(num_channels):
            # Determine which joint index to use
            if joint_names and ch < len(joint_names) and joint_names[ch]:
                joint_idx = self._find_joint_index(msg, joint_names[ch])
                if joint_idx < 0:
                    continue  # Joint not found in this message
            else:
                joint_idx = ch

            # Check bounds
            if joint_idx >= len(msg.position):
                continue

            angle = msg.position[joint_idx]
            pulse_us = self._angle_to_pulse_us(angle)
            encoder_count = self._angle_to_encoder_count(angle)

            channels.append((ch, pulse_us, encoder_count))

        if not channels:
            return

        # Build binary payload
        # Header: msg_type (1) + channel_count (1)
        # Per channel: channel (1) + pulse_us (2, LE) + encoder_count (4, LE)
        payload = struct.pack('<BB', MSG_TYPE_SERVO_CMD, len(channels))
        for ch, pulse, enc_count in channels:
            payload += struct.pack('<BHi', ch, pulse, enc_count)

        # Wrap in HILS frame protocol and send
        frame = frame_protocol.build_frame(payload)
        self.serial_write(frame)


def main(args=None):
    rclpy.init(args=args)
    node = PwmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
