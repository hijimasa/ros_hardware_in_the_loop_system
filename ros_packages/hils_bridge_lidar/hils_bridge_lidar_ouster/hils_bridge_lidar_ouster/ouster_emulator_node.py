#!/usr/bin/env python3
"""
Pure Software Ouster OS1 Emulator Node

Emulates an Ouster OS1 LiDAR entirely in software using a network interface
(e.g. USB-Ethernet adapter) assigned the LiDAR's IP address.
No custom hardware needed.

Implements the Ouster OS1 UDP packet protocol (simplified for HILS):
  - Lidar data packets: one column per UDP packet sent to host:7502
  - IMU packets: 48-byte packets sent to host:7503 at up to 100 Hz

Setup:
  # Assign OS1 IP to a USB-Ethernet adapter
  sudo ip addr add 192.168.1.100/24 dev eth1

  # Launch
  ros2 launch hils_bridge_lidar_ouster ouster_emulator.launch.py

Data flow:
  Simulation (PointCloud2/Imu) -> this node -> UDP (Ouster OS1) -> Robot PC
"""

import struct
import time

import numpy as np

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py import point_cloud2

from hils_bridge_base.udp_emulator_base import UdpEmulatorBase

# -- Ouster OS1 constants --

OUSTER_LIDAR_PORT = 7502
OUSTER_IMU_PORT = 7503

# Column header size in bytes
COLUMN_HEADER_SIZE = 16

# Channel data size per pixel in bytes
CHANNEL_DATA_SIZE = 12

# IMU packet size in bytes
IMU_PACKET_SIZE = 48

# OS1-16 beam altitude angles in degrees (ordered by channel index).
# These match the Ouster OS1-16 datasheet.
BEAM_ANGLES_16 = [
    16.611, 11.032, 5.579, 0.168, -5.125, -10.502, -15.880, -21.207,
    14.822, 9.206, 3.721, -1.673, -7.147, -12.526, -17.887, -23.339,
]

# Pre-compute beam angles in radians for vectorized operations
_BEAM_RAD_16 = np.array(BEAM_ANGLES_16, dtype=np.float64) * np.pi / 180.0


def _get_beam_angles(n_channels: int) -> np.ndarray:
    """Return beam altitude angles in degrees for the given channel count.

    For 16-channel mode, uses the standard OS1-16 angles.
    For 32/64-channel modes, generates evenly spaced angles covering
    the same vertical FOV as the OS1 sensor (~-23.3 to +16.6 deg).
    """
    if n_channels == 16:
        return np.array(BEAM_ANGLES_16, dtype=np.float64)
    # Approximate FOV for OS1 family
    fov_top = 16.6
    fov_bottom = -23.4
    return np.linspace(fov_top, fov_bottom, n_channels, dtype=np.float64)


def _build_column_packet(timestamp_ns: int, measurement_id: int,
                         frame_id: int, encoder_count: int,
                         channel_data: bytes) -> bytes:
    """Build a single Ouster lidar column packet.

    Args:
        timestamp_ns: Column timestamp in nanoseconds.
        measurement_id: Column index within the frame.
        frame_id: Frame counter (wraps at uint16 max).
        encoder_count: Azimuth encoder value (0-90111).
        channel_data: Packed channel data (n_channels * 12 bytes).

    Returns:
        Complete column packet as bytes.
    """
    header = struct.pack('<QHHI',
                         timestamp_ns,
                         measurement_id & 0xFFFF,
                         frame_id & 0xFFFF,
                         encoder_count & 0xFFFFFFFF)
    return header + channel_data


def _build_imu_packet(timestamp_ns: int,
                      accel_x: float, accel_y: float, accel_z: float,
                      gyro_x: float, gyro_y: float, gyro_z: float) -> bytes:
    """Build a 48-byte Ouster IMU packet.

    All three timestamp fields (diagnostic, accelerometer, gyroscope)
    are set to the same value for simplicity in HILS.

    Args:
        timestamp_ns: Timestamp in nanoseconds.
        accel_x, accel_y, accel_z: Accelerometer readings in m/s^2.
        gyro_x, gyro_y, gyro_z: Gyroscope readings in rad/s.

    Returns:
        48-byte IMU packet.
    """
    return struct.pack('<QQQffffff',
                       timestamp_ns,   # diagnostic_time
                       timestamp_ns,   # accelerometer_time
                       timestamp_ns,   # gyroscope_time
                       accel_x, accel_y, accel_z,
                       gyro_x, gyro_y, gyro_z)


class OusterEmulatorNode(UdpEmulatorBase):
    """ROS2 node that converts PointCloud2/Imu to Ouster OS1 UDP packets."""

    def __init__(self):
        super().__init__(
            node_name='hils_ouster_emulator',
            default_device_ip='192.168.1.100',
            default_host_ip='192.168.1.5',
        )

        # Additional parameters
        self.declare_parameter('pointcloud_topic', '/ouster/points',
            ParameterDescriptor(description='PointCloud2 topic from simulation'))
        self.declare_parameter('imu_topic', '/ouster/imu',
            ParameterDescriptor(description='Imu topic from simulation'))
        self.declare_parameter('enable_imu', True,
            ParameterDescriptor(description='Enable IMU packet output'))
        self.declare_parameter('max_points_per_frame', 30000,
            ParameterDescriptor(
                description='Max points per frame',
                integer_range=[IntegerRange(
                    from_value=100, to_value=300000, step=100)]))
        self.declare_parameter('downsample_mode', 'uniform',
            ParameterDescriptor(
                description='Downsample mode: "uniform" or "near"'))
        self.declare_parameter('lidar_mode', 512,
            ParameterDescriptor(
                description='Columns per frame (512, 1024, or 2048)',
                integer_range=[IntegerRange(
                    from_value=512, to_value=2048, step=512)]))
        self.declare_parameter('n_channels', 16,
            ParameterDescriptor(
                description='Number of vertical channels (16, 32, or 64)',
                integer_range=[IntegerRange(
                    from_value=16, to_value=64, step=16)]))

        self.add_on_set_parameters_callback(self._on_ouster_param_change)

        # Create UDP sockets via base class
        self._lidar_sock = self.create_device_socket(OUSTER_LIDAR_PORT)
        self._imu_sock = self.create_device_socket(OUSTER_IMU_PORT)

        self.get_logger().info(
            f'UDP sockets bound: lidar={OUSTER_LIDAR_PORT}, '
            f'imu={OUSTER_IMU_PORT}')

        # State
        self._frame_count = 0
        self._points_sent = 0
        self._imu_count = 0

        # Pre-compute beam angle lookup
        n_channels = self.get_parameter('n_channels').value
        self._beam_angles_deg = _get_beam_angles(n_channels)
        self._beam_angles_rad = self._beam_angles_deg * np.pi / 180.0

        # QoS for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        # Subscribe to PointCloud2
        pc_topic = self.get_parameter('pointcloud_topic').value
        self.create_subscription(
            PointCloud2, pc_topic, self._pointcloud_callback, sensor_qos)

        # Subscribe to IMU
        if self.get_parameter('enable_imu').value:
            imu_topic = self.get_parameter('imu_topic').value
            self.create_subscription(
                Imu, imu_topic, self._imu_callback, sensor_qos)

        lidar_mode = self.get_parameter('lidar_mode').value
        self.get_logger().info(
            f'Ouster OS1 Emulator started: topic={pc_topic}, '
            f'max_hz={self.get_parameter("max_hz").value}, '
            f'max_pts={self.get_parameter("max_points_per_frame").value}, '
            f'lidar_mode={lidar_mode}, n_channels={n_channels}, '
            f'downsample={self.get_parameter("downsample_mode").value}')

    def _on_ouster_param_change(self, params):
        for param in params:
            if param.name in ('max_points_per_frame', 'max_hz',
                              'downsample_mode', 'lidar_mode', 'n_channels'):
                self.get_logger().info(f'{param.name} changed to {param.value}')
                # Update beam angles if n_channels changes
                if param.name == 'n_channels':
                    self._beam_angles_deg = _get_beam_angles(param.value)
                    self._beam_angles_rad = (self._beam_angles_deg
                                             * np.pi / 180.0)
        return SetParametersResult(successful=True)

    # -- Point cloud processing --

    def _pointcloud_callback(self, msg: PointCloud2):
        """Convert PointCloud2 to Ouster OS1 column packets and send via UDP."""
        if not self.check_rate_limit():
            return

        max_pts = self.get_parameter('max_points_per_frame').value
        downsample_mode = self.get_parameter('downsample_mode').value
        lidar_mode = self.get_parameter('lidar_mode').value
        n_channels = self.get_parameter('n_channels').value

        try:
            packets = self._convert_to_ouster_packets(
                msg, max_pts, downsample_mode, lidar_mode, n_channels)
        except Exception as e:
            self.get_logger().warn(f'Point conversion failed: {e}')
            return

        if not packets:
            return

        host = (self.host_ip, OUSTER_LIDAR_PORT)
        for pkt in packets:
            try:
                self._lidar_sock.sendto(pkt, host)
            except OSError as e:
                self.get_logger().error(f'UDP send failed: {e}')
                return

        self.mark_sent()
        self._frame_count += 1

    def _convert_to_ouster_packets(self, msg: PointCloud2, max_pts: int,
                                    downsample_mode: str,
                                    lidar_mode: int,
                                    n_channels: int) -> list:
        """Convert a PointCloud2 message to a list of Ouster column packets.

        Steps:
          1. Extract x, y, z, intensity from the point cloud
          2. Downsample if exceeding max_pts
          3. Convert Cartesian to spherical (azimuth, elevation, range)
          4. Map elevation to nearest OS1 channel index
          5. Quantize azimuth into column bins
          6. Pack each column as a UDP packet with channel data

        Returns:
            List of bytes objects, each one Ouster column packet.
        """
        # Extract points
        field_names = ['x', 'y', 'z']
        has_intensity = any(f.name in ('intensity', 'reflectivity')
                           for f in msg.fields)
        if has_intensity:
            int_field = ('intensity' if any(f.name == 'intensity'
                         for f in msg.fields) else 'reflectivity')
            field_names.append(int_field)

        pts_arr = point_cloud2.read_points(
            msg, field_names=field_names, skip_nans=True)
        if pts_arr is None or len(pts_arr) == 0:
            return []

        n_pts = len(pts_arr)

        # Downsample if needed
        if n_pts > max_pts:
            if downsample_mode == 'near':
                dist_sq = (pts_arr['x'].astype(np.float64) ** 2 +
                           pts_arr['y'].astype(np.float64) ** 2 +
                           pts_arr['z'].astype(np.float64) ** 2)
                indices = np.argpartition(dist_sq, max_pts)[:max_pts]
                pts_arr = pts_arr[indices]
            else:  # uniform
                indices = np.linspace(0, n_pts - 1, max_pts, dtype=np.intp)
                pts_arr = pts_arr[indices]
            n_pts = max_pts

        # Extract coordinates as float64
        x = pts_arr['x'].astype(np.float64)
        y = pts_arr['y'].astype(np.float64)
        z = pts_arr['z'].astype(np.float64)

        # Reflectivity (0-65535 for uint16)
        if has_intensity:
            refl = np.clip(pts_arr[int_field], 0, 65535).astype(np.uint16)
        else:
            refl = np.zeros(n_pts, dtype=np.uint16)

        # Convert to spherical coordinates
        xy_dist = np.sqrt(x * x + y * y)
        distance = np.sqrt(x * x + y * y + z * z)

        # Azimuth: atan2(y, x) -> degrees [0, 360)
        azimuth_deg = np.degrees(np.arctan2(y, x)) % 360.0

        # Elevation: atan2(z, xy_dist) -> degrees
        elevation_deg = np.degrees(np.arctan2(z, xy_dist))

        # Map each point to nearest OS1 channel
        beam_angles = self._beam_angles_deg
        elev_diff = np.abs(
            elevation_deg[:, np.newaxis] - beam_angles[np.newaxis, :])
        channel_idx = np.argmin(elev_diff, axis=1).astype(np.uint16)

        # Range in millimeters (uint32)
        range_mm = np.clip(distance * 1000.0, 0, 0xFFFFFFFF).astype(np.uint32)

        # Azimuth quantized to column bins
        # Encoder count: 0-90111 maps to 0-360 degrees
        encoder_per_deg = 90112.0 / 360.0
        encoder_count = (azimuth_deg * encoder_per_deg).astype(np.uint32)

        # Column index: quantize azimuth into lidar_mode bins
        col_idx = ((azimuth_deg / 360.0) * lidar_mode).astype(np.int32)
        col_idx = np.clip(col_idx, 0, lidar_mode - 1)

        # Sort by column index for sequential packet output
        sort_idx = np.argsort(col_idx)
        col_idx = col_idx[sort_idx]
        channel_idx = channel_idx[sort_idx]
        range_mm = range_mm[sort_idx]
        refl = refl[sort_idx]
        encoder_count = encoder_count[sort_idx]

        # Build column packets
        timestamp_ns = (msg.header.stamp.sec * 1_000_000_000
                        + msg.header.stamp.nanosec)
        frame_id = self._frame_count & 0xFFFF

        # Time increment per column
        # At 10 Hz with lidar_mode columns: 100ms / lidar_mode per column
        col_time_ns = 100_000_000 // lidar_mode  # nanoseconds per column

        packets = self._pack_columns(
            col_idx, channel_idx, range_mm, refl, encoder_count,
            n_channels, lidar_mode, timestamp_ns, frame_id, col_time_ns)

        return packets

    def _pack_columns(self, col_idx, channel_idx, range_mm, refl,
                      encoder_count, n_channels, lidar_mode,
                      timestamp_ns, frame_id, col_time_ns) -> list:
        """Pack sorted point data into Ouster column packets.

        Groups points by column index. For each populated column, builds
        a packet with the column header and n_channels channel data entries.
        Empty channels are filled with zeros.
        """
        n_pts = len(col_idx)
        if n_pts == 0:
            return []

        # Find unique columns that have data
        unique_cols = np.unique(col_idx)
        packets = []

        # Pre-allocate channel data buffer (reused per column)
        channel_bytes_size = n_channels * CHANNEL_DATA_SIZE

        for col in unique_cols:
            mask = col_idx == col
            c_channels = channel_idx[mask]
            c_range = range_mm[mask]
            c_refl = refl[mask]
            c_encoder = encoder_count[mask]

            # Build channel data (n_channels * 12 bytes)
            channel_data = bytearray(channel_bytes_size)

            # Fill channels that have data. For duplicate channels in the
            # same column, keep the closest return.
            for i in range(len(c_channels)):
                ch = int(c_channels[i])
                if ch >= n_channels:
                    continue
                offset = ch * CHANNEL_DATA_SIZE
                existing_range = struct.unpack_from('<I', channel_data, offset)[0]
                new_range = int(c_range[i])
                if existing_range == 0 or new_range < existing_range:
                    struct.pack_into('<I', channel_data, offset, new_range)
                    struct.pack_into('<H', channel_data, offset + 4, int(c_refl[i]))
                    struct.pack_into('<H', channel_data, offset + 6, int(c_refl[i]))
                    struct.pack_into('<H', channel_data, offset + 8, 0)  # near_ir
                    struct.pack_into('<H', channel_data, offset + 10, 0)  # reserved

            # Column timestamp
            col_ts = timestamp_ns + int(col) * col_time_ns

            # Encoder count: median of points in this column
            median_encoder = int(np.median(c_encoder))

            pkt = _build_column_packet(
                col_ts, int(col), frame_id, median_encoder,
                bytes(channel_data))
            packets.append(pkt)
            self._points_sent += int(mask.sum())

        return packets

    # -- IMU processing --

    def _imu_callback(self, msg: Imu):
        """Convert ROS Imu message to Ouster IMU UDP packet and send."""
        timestamp_ns = (msg.header.stamp.sec * 1_000_000_000
                        + msg.header.stamp.nanosec)
        pkt = _build_imu_packet(
            timestamp_ns,
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z)
        try:
            self._imu_sock.sendto(pkt, (self.host_ip, OUSTER_IMU_PORT))
            self._imu_count += 1
        except OSError as e:
            self.get_logger().debug(f'IMU packet send failed: {e}')

    # -- Stats override --

    def _stats_callback(self):
        self.get_logger().info(
            f'Status: frames={self._frame_count}, '
            f'points_sent={self._points_sent}, '
            f'imu_pkts={self._imu_count}, '
            f'device_ip={self.device_ip}')

    # -- Cleanup --

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OusterEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
