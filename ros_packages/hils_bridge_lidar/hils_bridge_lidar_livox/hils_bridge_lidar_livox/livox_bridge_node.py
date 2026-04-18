#!/usr/bin/env python3
"""
HILS Livox Bridge Node

Subscribes to PointCloud2 and IMU topics from simulation, converts to
Livox Mid360 point format, and sends to W5500-EVB-Pico2 via USB CDC
using the HILS framing protocol.

The firmware then emulates a Livox Mid360 on Ethernet using the
Livox SDK2 protocol, allowing livox_ros_driver2 on the robot PC
to receive the data as if from a real LiDAR.

Data flow:
    Simulation (PointCloud2/IMU) -> this node -> USB CDC -> W5500-EVB-Pico2
    -> Ethernet UDP (Livox SDK2) -> Robot PC (livox_ros_driver2)
"""

import struct
import time
import threading

import numpy as np
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2

from hils_bridge_lidar_livox import frame_protocol


class LivoxBridgeNode(Node):
    def __init__(self):
        super().__init__('hils_livox_bridge')

        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('pointcloud_topic', '/livox/lidar')
        self.declare_parameter('imu_topic', '/livox/imu')
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('max_hz', 10.0)
        self.declare_parameter('max_points_per_frame', 4600,
            ParameterDescriptor(
                description='Max points per HILS frame (USB bandwidth limit)',
                integer_range=[IntegerRange(from_value=100, to_value=20000, step=100)]))
        self.declare_parameter('point_data_type', 1,
            ParameterDescriptor(
                description='1=CartesianHigh(14B/pt), 2=CartesianLow(8B/pt)',
                integer_range=[IntegerRange(from_value=1, to_value=2, step=1)]))
        self.declare_parameter('downsample_mode', 'uniform',
            ParameterDescriptor(
                description='Downsample mode: "uniform" (even spacing) or "near" (closest points first)'))
        self.declare_parameter('lidar_ip', '192.168.1.12')
        self.declare_parameter('host_ip', '192.168.1.5')
        self.declare_parameter('serial_number', '0TFDFH600100511')

        self.add_on_set_parameters_callback(self._on_param_change)

        # Open serial port
        port = self.get_parameter('serial_port').value
        try:
            self.serial = serial.Serial(port, timeout=0)
            self.get_logger().info(f'Opened serial port: {port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {port}: {e}')
            raise

        self._serial_write_lock = threading.Lock()

        # Send initial configuration to firmware
        self._send_config()

        # QoS: BEST_EFFORT to match typical sensor/simulation publishers
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscribe to point cloud topic
        pc_topic = self.get_parameter('pointcloud_topic').value
        self.pc_sub = self.create_subscription(
            PointCloud2, pc_topic, self._pointcloud_callback, sensor_qos)

        # Subscribe to IMU topic
        if self.get_parameter('enable_imu').value:
            imu_topic = self.get_parameter('imu_topic').value
            self.imu_sub = self.create_subscription(
                Imu, imu_topic, self._imu_callback, sensor_qos)

        self.last_send_time = 0.0
        self.frame_count = 0

        # Start reverse-channel read thread
        self._receiver = frame_protocol.FrameProtocolReceiver()
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info(
            f'Livox Bridge started: pc_topic={pc_topic}, '
            f'max_hz={self.get_parameter("max_hz").value}, '
            f'max_pts={self.get_parameter("max_points_per_frame").value}')

    def _on_param_change(self, params):
        for param in params:
            if param.name in ('max_points_per_frame', 'point_data_type',
                              'max_hz', 'downsample_mode'):
                self.get_logger().info(f'{param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

    def _send_config(self):
        """Send network configuration to firmware."""
        lidar_ip = self.get_parameter('lidar_ip').value
        host_ip = self.get_parameter('host_ip').value
        sn = self.get_parameter('serial_number').value

        frames = [
            frame_protocol.build_set_ip_frame(frame_protocol.CFG_SET_LIDAR_IP, lidar_ip),
            frame_protocol.build_set_ip_frame(frame_protocol.CFG_SET_HOST_IP, host_ip),
            frame_protocol.build_set_sn_frame(sn),
            frame_protocol.build_config_frame(frame_protocol.CFG_START),
        ]

        with self._serial_write_lock:
            for f in frames:
                self.serial.write(f)
            self.serial.flush()

        self.get_logger().info(
            f'Config sent: lidar_ip={lidar_ip}, host_ip={host_ip}, sn={sn}')

    def _serial_read_loop(self):
        """Background thread: reads status reports from firmware."""
        while rclpy.ok():
            try:
                data = self.serial.read(256)
                if data:
                    for payload in self._receiver.feed(data):
                        self._handle_status(payload)
                else:
                    time.sleep(0.01)
            except serial.SerialException:
                break

    def _handle_status(self, payload: bytes):
        result = frame_protocol.parse_status(payload)
        if result:
            state, points_sent, udp_errors = result
            state_names = {0: 'idle', 1: 'ready', 2: 'streaming'}
            self.get_logger().info(
                f'Firmware status: {state_names.get(state, "unknown")}, '
                f'points_sent={points_sent}, udp_errors={udp_errors}')

    def _pointcloud_callback(self, msg: PointCloud2):
        # Rate limiting
        now = time.monotonic()
        min_interval = 1.0 / self.get_parameter('max_hz').value
        if (now - self.last_send_time) < min_interval:
            return

        self.get_logger().debug(
            f'PC2 received: fields={[f.name for f in msg.fields]}, '
            f'width={msg.width}, height={msg.height}, '
            f'point_step={msg.point_step}, row_step={msg.row_step}')

        data_type = self.get_parameter('point_data_type').value
        max_pts = self.get_parameter('max_points_per_frame').value
        downsample_mode = self.get_parameter('downsample_mode').value

        # Extract points from PointCloud2
        try:
            points = self._extract_points(msg, max_pts, data_type, downsample_mode)
        except Exception as e:
            self.get_logger().warn(f'Point extraction failed: {e}')
            return

        if points is None:
            self.get_logger().warn('No points extracted (all NaN?)')
            return

        point_data, dot_num = points

        # Timestamp from message header (nanoseconds)
        timestamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        # Build and send frame(s), splitting if payload exceeds max
        point_size = (frame_protocol.POINT_SIZE_HIGH
                      if data_type == frame_protocol.DATA_TYPE_CARTESIAN_HIGH
                      else frame_protocol.POINT_SIZE_LOW)
        # Max points per frame: (MAX_PAYLOAD - header 12 bytes) / point_size
        max_pts_per_frame = (frame_protocol.MAX_PAYLOAD - 12) // point_size

        try:
            offset = 0
            remaining = dot_num
            while remaining > 0:
                chunk = min(remaining, max_pts_per_frame)
                chunk_data = point_data[offset:offset + chunk * point_size]
                frame = frame_protocol.build_points_frame(
                    data_type, chunk, timestamp_ns, chunk_data)
                with self._serial_write_lock:
                    self.serial.write(frame)
                    self.serial.flush()
                offset += chunk * point_size
                remaining -= chunk
                timestamp_ns += int(chunk * 4.7e3)  # advance timestamp
        except (serial.SerialException, ValueError) as e:
            self.get_logger().error(f'Serial write failed: {e}')
            return

        self.last_send_time = now
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Sent {self.frame_count} frames '
                f'(pts={dot_num}, type={data_type}, '
                f'payload={len(point_data)} bytes)')

    def _extract_points(self, msg: PointCloud2, max_pts: int, data_type: int,
                        downsample_mode: str = 'uniform'):
        """Extract points from PointCloud2 and pack into Livox binary format.

        Uses numpy vectorized operations for speed.
        """
        # Determine fields to read
        field_names = ['x', 'y', 'z']
        has_intensity = any(f.name in ('intensity', 'reflectivity')
                           for f in msg.fields)
        if has_intensity:
            int_field = 'intensity' if any(
                f.name == 'intensity' for f in msg.fields) else 'reflectivity'
            field_names.append(int_field)

        # Read all points as structured numpy array
        pts_arr = point_cloud2.read_points(msg, field_names=field_names,
                                           skip_nans=True)
        if pts_arr is None or len(pts_arr) == 0:
            return None

        n_pts = len(pts_arr)
        if n_pts == 0:
            return None

        # Downsample if needed
        if n_pts > max_pts:
            if downsample_mode == 'near':
                dist_sq = (pts_arr['x'].astype(np.float64) ** 2 +
                           pts_arr['y'].astype(np.float64) ** 2 +
                           pts_arr['z'].astype(np.float64) ** 2)
                indices = np.argpartition(dist_sq, max_pts)[:max_pts]
            else:  # uniform
                indices = np.linspace(0, n_pts - 1, max_pts, dtype=np.intp)
            pts_arr = pts_arr[indices]
            n_pts = max_pts

        # Get reflectivity
        if has_intensity:
            refl = np.clip(pts_arr[int_field], 0, 255).astype(np.uint8)
        else:
            refl = np.zeros(n_pts, dtype=np.uint8)
        tag = np.zeros(n_pts, dtype=np.uint8)

        if data_type == frame_protocol.DATA_TYPE_CARTESIAN_HIGH:
            # int32 mm: x,y,z + uint8 reflectivity + uint8 tag = 14 bytes/pt
            x = (pts_arr['x'] * 1000.0).astype(np.int32)
            y = (pts_arr['y'] * 1000.0).astype(np.int32)
            z = (pts_arr['z'] * 1000.0).astype(np.int32)
            # Pack into structured array (14 bytes/point, no padding)
            dt = np.dtype([('x', '<i4'), ('y', '<i4'), ('z', '<i4'),
                           ('r', 'u1'), ('t', 'u1')])
            arr = np.empty(n_pts, dtype=dt)
            arr['x'] = x
            arr['y'] = y
            arr['z'] = z
            arr['r'] = refl
            arr['t'] = tag
            return arr.tobytes(), n_pts
        else:
            # int16 cm: x,y,z + uint8 reflectivity + uint8 tag = 8 bytes/pt
            x = np.clip((pts_arr['x'] * 100.0).astype(np.int32),
                        -32768, 32767).astype(np.int16)
            y = np.clip((pts_arr['y'] * 100.0).astype(np.int32),
                        -32768, 32767).astype(np.int16)
            z = np.clip((pts_arr['z'] * 100.0).astype(np.int32),
                        -32768, 32767).astype(np.int16)
            dt = np.dtype([('x', '<i2'), ('y', '<i2'), ('z', '<i2'),
                           ('r', 'u1'), ('t', 'u1')])
            arr = np.empty(n_pts, dtype=dt)
            arr['x'] = x
            arr['y'] = y
            arr['z'] = z
            arr['r'] = refl
            arr['t'] = tag
            return arr.tobytes(), n_pts

    def _imu_callback(self, msg: Imu):
        """Forward IMU data to firmware."""
        try:
            frame = frame_protocol.build_imu_frame(
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z)
            with self._serial_write_lock:
                self.serial.write(frame)
                self.serial.flush()
        except (serial.SerialException, ValueError) as e:
            self.get_logger().error(f'IMU serial write failed: {e}')

    def destroy_node(self):
        # Send stop command
        try:
            if hasattr(self, 'serial') and self.serial.is_open:
                stop_frame = frame_protocol.build_config_frame(frame_protocol.CFG_STOP)
                with self._serial_write_lock:
                    self.serial.write(stop_frame)
                    self.serial.flush()
                self.serial.close()
        except serial.SerialException:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LivoxBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
