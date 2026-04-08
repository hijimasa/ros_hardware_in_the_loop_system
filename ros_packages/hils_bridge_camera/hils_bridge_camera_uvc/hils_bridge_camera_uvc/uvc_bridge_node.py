#!/usr/bin/env python3
"""
HILS UVC Bridge Node

Subscribes to a ROS image topic, JPEG-encodes the frames, and sends them
to Pico#1 via USB CDC serial using the HILS framing protocol.

Receives resolution change commands from the UVC host (via Pico#2 -> Pico#1)
and dynamically adjusts the output resolution.

Data flow:
    /image_raw -> JPEG encode -> frame protocol -> serial -> Pico#1 -> UART -> Pico#2 -> UVC
    UVC host -> Pico#2 -> UART -> Pico#1 -> serial -> this node (resolution update)

JPEG quality and resolution can be changed at runtime:
    ros2 param set /hils_uvc_bridge jpeg_quality 80
"""

import cv2
import serial
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from hils_bridge_camera_uvc import frame_protocol


class UvcBridgeNode(Node):
    def __init__(self):
        super().__init__('hils_uvc_bridge')

        # Parameters with descriptors for discoverability
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('jpeg_quality', 50,
            ParameterDescriptor(
                description='JPEG encoding quality (1-100). Higher = better quality, larger frames.',
                integer_range=[IntegerRange(from_value=1, to_value=100, step=1)]))
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        # Parameter change callback
        self.add_on_set_parameters_callback(self._on_param_change)

        # Open serial port to Pico#1
        port = self.get_parameter('serial_port').value
        try:
            self.serial = serial.Serial(port, timeout=0)
            self.get_logger().info(f'Opened serial port: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        # Serial write lock (image_callback writes, read thread reads)
        self._serial_write_lock = threading.Lock()

        # Subscribe to image topic
        topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(
            Image, topic, self.image_callback, 1)  # queue_size=1: drop old frames

        self.bridge = CvBridge()
        self.last_send_time = 0.0
        self.frame_count = 0

        # Start reverse-channel read thread
        self._receiver = frame_protocol.FrameProtocolReceiver()
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info(
            f'UVC Bridge started: topic={topic}, quality={self.get_parameter("jpeg_quality").value}, '
            f'max_fps={self.get_parameter("max_fps").value}')

    def _on_param_change(self, params):
        for param in params:
            if param.name in ('jpeg_quality', 'frame_width', 'frame_height'):
                self.get_logger().info(f'{param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

    def _serial_read_loop(self):
        """Background thread: reads reverse-channel commands from Pico#1."""
        while rclpy.ok():
            try:
                data = self.serial.read(256)
                if data:
                    for payload in self._receiver.feed(data):
                        self._handle_command(payload)
                else:
                    time.sleep(0.01)
            except serial.SerialException:
                break

    def _handle_command(self, payload: bytes):
        """Process a reverse-channel command."""
        result = frame_protocol.parse_resolution_cmd(payload)
        if result:
            width, height, frame_index = result
            self.get_logger().info(
                f'UVC host selected resolution: {width}x{height} (index={frame_index})')
            self.set_parameters([
                Parameter('frame_width', Parameter.Type.INTEGER, width),
                Parameter('frame_height', Parameter.Type.INTEGER, height),
            ])

    def image_callback(self, msg: Image):
        # Rate limiting
        now = time.monotonic()
        min_interval = 1.0 / self.get_parameter('max_fps').value
        if (now - self.last_send_time) < min_interval:
            return

        # Convert ROS Image to OpenCV BGR
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed: {e}')
            return

        # Resize to match UVC descriptor resolution
        target_w = self.get_parameter('frame_width').value
        target_h = self.get_parameter('frame_height').value
        h, w = cv_image.shape[:2]
        if w != target_w or h != target_h:
            cv_image = cv2.resize(cv_image, (target_w, target_h))

        # JPEG encode
        quality = self.get_parameter('jpeg_quality').value
        ret, jpeg_buf = cv2.imencode(
            '.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ret:
            self.get_logger().warn('JPEG encode failed')
            return

        jpeg_bytes = jpeg_buf.tobytes()

        # Build framed packet and send
        try:
            frame = frame_protocol.build_frame(jpeg_bytes)
            with self._serial_write_lock:
                self.serial.write(frame)
                self.serial.flush()
        except (serial.SerialException, ValueError) as e:
            self.get_logger().error(f'Serial write failed: {e}')
            return

        self.last_send_time = now
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Sent {self.frame_count} frames '
                f'(quality={quality}, {target_w}x{target_h}, '
                f'JPEG={len(jpeg_bytes)} bytes)')

    def destroy_node(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UvcBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
