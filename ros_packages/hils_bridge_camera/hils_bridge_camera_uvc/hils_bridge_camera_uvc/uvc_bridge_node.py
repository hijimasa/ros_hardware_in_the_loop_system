#!/usr/bin/env python3
"""
HILS UVC Bridge Node

Subscribes to a ROS image topic, JPEG-encodes the frames, and sends them
to Pico#1 via USB CDC serial using the HILS framing protocol.

Data flow:
    /image_raw -> JPEG encode -> frame protocol -> serial -> Pico#1 -> SPI -> Pico#2 -> UVC
"""

import cv2
import serial
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from hils_bridge_camera_uvc import frame_protocol


class UvcBridgeNode(Node):
    def __init__(self):
        super().__init__('hils_uvc_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('jpeg_quality', 50)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        # Open serial port to Pico#1
        port = self.get_parameter('serial_port').value
        try:
            self.serial = serial.Serial(port, timeout=0)
            self.get_logger().info(f'Opened serial port: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        # Subscribe to image topic
        topic = self.get_parameter('image_topic').value
        self.sub = self.create_subscription(
            Image, topic, self.image_callback, 1)  # queue_size=1: drop old frames

        self.bridge = CvBridge()
        self.last_send_time = 0.0
        self.frame_count = 0

        self.get_logger().info(
            f'UVC Bridge started: topic={topic}, quality={self.get_parameter("jpeg_quality").value}, '
            f'max_fps={self.get_parameter("max_fps").value}')

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
                f'(src: {w}x{h}, out: {target_w}x{target_h}, '
                f'JPEG: {len(jpeg_bytes)} bytes)')

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
