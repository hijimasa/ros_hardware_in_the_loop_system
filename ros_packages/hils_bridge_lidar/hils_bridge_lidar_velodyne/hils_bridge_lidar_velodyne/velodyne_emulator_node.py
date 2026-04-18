#!/usr/bin/env python3
"""
Pure Software Velodyne VLP-16 Emulator Node

Emulates a Velodyne VLP-16 LiDAR entirely in software using a network interface
(e.g. USB-Ethernet adapter) assigned the LiDAR's IP address.
No custom hardware needed.

Implements the Velodyne VLP-16 UDP packet protocol:
  - Data packets: 12 data blocks per packet sent to host:2368
  - Position packets: minimal valid packets sent to host:8308 periodically

Setup:
  # Assign VLP-16 IP to a USB-Ethernet adapter
  sudo ip addr add 192.168.1.201/24 dev eth1

  # Launch
  ros2 launch hils_bridge_lidar_velodyne velodyne_emulator.launch.py

Data flow:
  Simulation (PointCloud2) -> this node -> UDP (Velodyne VLP-16) -> Robot PC
"""

import struct
import time

import numpy as np

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor, IntegerRange, SetParametersResult
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from hils_bridge_base.udp_emulator_base import UdpEmulatorBase

# -- VLP-16 constants --

VLP16_DATA_PORT = 2368
VLP16_POSITION_PORT = 8308

VLP16_BLOCK_FLAG = 0xFFEE
VLP16_PRODUCT_ID = 0x22

# Return modes
RETURN_STRONGEST = 0x37
RETURN_LAST = 0x38
RETURN_DUAL = 0x39

# Packet structure sizes
VLP16_BLOCK_SIZE = 100        # 2 flag + 2 azimuth + 96 channel data
VLP16_BLOCKS_PER_PACKET = 12
VLP16_DATA_BLOCK_TOTAL = VLP16_BLOCK_SIZE * VLP16_BLOCKS_PER_PACKET  # 1200
VLP16_PACKET_SIZE = 1248      # 1200 data + 4 timestamp + 2 factory

# Each block has 32 channels (2 firing sequences x 16 lasers), 3 bytes each
VLP16_CHANNELS_PER_BLOCK = 32
VLP16_CHANNEL_DATA_SIZE = 3   # uint16 distance + uint8 reflectivity
VLP16_LASERS = 16

# VLP-16 channel elevation angles in degrees, indexed by laser id (0-15).
# The firing order interleaves: channel 0 = -15 deg, channel 1 = +1 deg, etc.
CHANNEL_ELEVATIONS = [
    -15.0, 1.0, -13.0, 3.0, -11.0, 5.0, -9.0, 7.0,
    -7.0, 9.0, -5.0, 11.0, -3.0, 13.0, -1.0, 15.0
]

# Pre-compute elevation angles in radians for vectorized operations
_ELEV_RAD = np.array(CHANNEL_ELEVATIONS, dtype=np.float64) * np.pi / 180.0

# Position packet is 512 bytes, mostly zeros for HILS
VLP16_POSITION_PACKET_SIZE = 512


def _build_empty_block(azimuth_cdeg: int) -> bytearray:
    """Build a data block with all zero distances (no returns)."""
    block = bytearray(VLP16_BLOCK_SIZE)
    struct.pack_into('<H', block, 0, VLP16_BLOCK_FLAG)
    struct.pack_into('<H', block, 2, azimuth_cdeg % 36000)
    return block


def _build_position_packet(timestamp_us: int) -> bytes:
    """Build a minimal valid position (GPRMC) packet.

    The position packet is 512 bytes. For HILS purposes we only fill
    the timestamp and leave the rest zeroed (no GPS fix).
    """
    pkt = bytearray(VLP16_POSITION_PACKET_SIZE)
    # Bytes 198-199: reserved (unused in many drivers)
    # Bytes 200-205: NMEA sentence placeholder (empty)
    # Bytes 198-201: timestamp (microseconds since top of hour)
    struct.pack_into('<I', pkt, 198, timestamp_us & 0xFFFFFFFF)
    return bytes(pkt)


class VelodyneEmulatorNode(UdpEmulatorBase):
    """ROS2 node that converts PointCloud2 to Velodyne VLP-16 UDP packets."""

    def __init__(self):
        super().__init__(
            node_name='hils_velodyne_emulator',
            default_device_ip='192.168.1.201',
            default_host_ip='192.168.1.100',
        )

        # Additional parameters
        self.declare_parameter('pointcloud_topic', '/velodyne_points',
            ParameterDescriptor(description='PointCloud2 topic from simulation'))
        self.declare_parameter('max_points_per_frame', 30000,
            ParameterDescriptor(
                description='Max points per frame',
                integer_range=[IntegerRange(
                    from_value=100, to_value=300000, step=100)]))
        self.declare_parameter('return_mode', RETURN_STRONGEST,
            ParameterDescriptor(
                description='Return mode: 0x37=Strongest, 0x38=Last, 0x39=Dual',
                integer_range=[IntegerRange(
                    from_value=RETURN_STRONGEST, to_value=RETURN_DUAL, step=1)]))
        self.declare_parameter('downsample_mode', 'uniform',
            ParameterDescriptor(
                description='Downsample mode: "uniform" or "near"'))

        self.add_on_set_parameters_callback(self._on_velodyne_param_change)

        # Create UDP sockets via base class
        self._data_sock = self.create_device_socket(VLP16_DATA_PORT)
        self._position_sock = self.create_device_socket(VLP16_POSITION_PORT)

        self.get_logger().info(
            f'UDP sockets bound: data={VLP16_DATA_PORT}, '
            f'position={VLP16_POSITION_PORT}')

        # State
        self._frame_count = 0
        self._points_sent = 0

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

        # Position packet timer (every 0.5s, matching real VLP-16)
        self.create_timer(0.5, self._position_timer_callback)

        self.get_logger().info(
            f'Velodyne VLP-16 Emulator started: topic={pc_topic}, '
            f'max_hz={self.get_parameter("max_hz").value}, '
            f'max_pts={self.get_parameter("max_points_per_frame").value}, '
            f'downsample={self.get_parameter("downsample_mode").value}')

    def _on_velodyne_param_change(self, params):
        for param in params:
            if param.name in ('max_points_per_frame', 'return_mode',
                              'max_hz', 'downsample_mode'):
                self.get_logger().info(f'{param.name} changed to {param.value}')
        return SetParametersResult(successful=True)

    # -- Position packet --

    def _position_timer_callback(self):
        """Send periodic position packets to host:8308."""
        now = time.time()
        # Microseconds since top of hour
        secs_in_hour = now % 3600.0
        timestamp_us = int(secs_in_hour * 1_000_000)
        pkt = _build_position_packet(timestamp_us)
        try:
            self._position_sock.sendto(
                pkt, (self.host_ip, VLP16_POSITION_PORT))
        except OSError as e:
            self.get_logger().debug(f'Position packet send failed: {e}')

    # -- Point cloud processing --

    def _pointcloud_callback(self, msg: PointCloud2):
        """Convert PointCloud2 to VLP-16 data packets and send via UDP."""
        if not self.check_rate_limit():
            return

        max_pts = self.get_parameter('max_points_per_frame').value
        return_mode = self.get_parameter('return_mode').value
        downsample_mode = self.get_parameter('downsample_mode').value

        try:
            packets = self._convert_to_vlp16_packets(
                msg, max_pts, return_mode, downsample_mode)
        except Exception as e:
            self.get_logger().warn(f'Point conversion failed: {e}')
            return

        if not packets:
            return

        host = (self.host_ip, VLP16_DATA_PORT)
        for pkt in packets:
            try:
                self._data_sock.sendto(pkt, host)
            except OSError as e:
                self.get_logger().error(f'UDP send failed: {e}')
                return

        self.mark_sent()
        self._frame_count += 1

    def _convert_to_vlp16_packets(self, msg: PointCloud2, max_pts: int,
                                   return_mode: int,
                                   downsample_mode: str) -> list:
        """Convert a PointCloud2 message to a list of VLP-16 UDP packets.

        Steps:
          1. Extract x, y, z, intensity from the point cloud
          2. Convert Cartesian to spherical (azimuth, elevation, distance)
          3. Map elevation to VLP-16 channel index (nearest match)
          4. Sort by azimuth, group into data blocks
          5. Pack blocks into 1248-byte packets (12 blocks each)

        Returns:
            List of bytes objects, each 1248 bytes (VLP-16 data packet).
        """
        # Extract points
        field_names = ['x', 'y', 'z']
        has_intensity = any(f.name in ('intensity', 'reflectivity')
                           for f in msg.fields)
        if has_intensity:
            int_field = 'intensity' if any(
                f.name == 'intensity' for f in msg.fields) else 'reflectivity'
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

        # Reflectivity (0-255)
        if has_intensity:
            refl = np.clip(pts_arr[int_field], 0, 255).astype(np.uint8)
        else:
            refl = np.zeros(n_pts, dtype=np.uint8)

        # Convert to spherical coordinates
        xy_dist = np.sqrt(x * x + y * y)
        distance = np.sqrt(x * x + y * y + z * z)

        # Azimuth: atan2(y, x), convert to degrees [0, 360)
        # Velodyne convention: 0 deg = +Y axis, CW when viewed from above
        # ROS convention: atan2(y, x) gives angle from +X axis, CCW
        # Velodyne azimuth = 90 - atan2(y,x) in degrees, wrapped to [0, 360)
        azimuth_deg = np.degrees(np.arctan2(y, x))
        # Convert from math convention to Velodyne convention (CW from +Y)
        azimuth_deg = (90.0 - azimuth_deg) % 360.0

        # Elevation: atan2(z, xy_dist), convert to degrees
        elevation_deg = np.degrees(np.arctan2(z, xy_dist))

        # Map each point to nearest VLP-16 channel
        elev_arr = np.array(CHANNEL_ELEVATIONS, dtype=np.float64)
        # Shape: (n_pts, 16) -> difference from each channel
        elev_diff = np.abs(elevation_deg[:, np.newaxis] - elev_arr[np.newaxis, :])
        channel_idx = np.argmin(elev_diff, axis=1).astype(np.uint8)

        # Distance in 2mm units (uint16), clamp to valid range
        dist_2mm = np.clip((distance / 0.002), 0, 65535).astype(np.uint16)

        # Azimuth in 0.01 degree units (uint16)
        azimuth_cdeg = np.clip((azimuth_deg * 100.0), 0, 35999).astype(np.uint16)

        # Sort by azimuth for proper packet ordering
        sort_idx = np.argsort(azimuth_cdeg)
        azimuth_cdeg = azimuth_cdeg[sort_idx]
        channel_idx = channel_idx[sort_idx]
        dist_2mm = dist_2mm[sort_idx]
        refl = refl[sort_idx]

        # Build data blocks
        # Each block covers a small azimuth range and has 32 channels
        # (2 firing sequences x 16 lasers).
        # We quantize azimuth to create blocks, with each block having a
        # single azimuth value and up to 16 channel slots per firing.
        packets = self._pack_into_packets(
            azimuth_cdeg, channel_idx, dist_2mm, refl, return_mode, msg)
        return packets

    def _pack_into_packets(self, azimuth_cdeg, channel_idx, dist_2mm,
                           refl, return_mode, msg) -> list:
        """Pack sorted point data into VLP-16 data packets.

        Groups points by azimuth into data blocks. Each block holds two
        firing sequences of 16 lasers. We create blocks at discrete azimuth
        steps, fill in channels that have data, and pack 12 blocks per packet.
        """
        n_pts = len(azimuth_cdeg)
        if n_pts == 0:
            return []

        # Determine timestamp (microseconds since top of hour)
        stamp_sec = msg.header.stamp.sec
        stamp_nsec = msg.header.stamp.nanosec
        epoch_sec = float(stamp_sec) + float(stamp_nsec) * 1e-9
        secs_in_hour = epoch_sec % 3600.0
        base_timestamp_us = int(secs_in_hour * 1_000_000)

        # Quantize azimuth: group points that share similar azimuths.
        # VLP-16 real azimuth step is ~0.2 deg between firings.
        # We use 0.4 deg (40 cdeg) per block to match 2 firings per block.
        AZIMUTH_STEP_CDEG = 40  # 0.4 degrees per block

        # Create block assignments
        block_id = azimuth_cdeg // AZIMUTH_STEP_CDEG

        # Find unique blocks and iterate
        unique_blocks = np.unique(block_id)

        # Pre-allocate block list
        blocks = []

        for bid in unique_blocks:
            mask = block_id == bid
            blk_az = int(np.median(azimuth_cdeg[mask]))
            blk_ch = channel_idx[mask]
            blk_dist = dist_2mm[mask]
            blk_refl = refl[mask]

            block = bytearray(VLP16_BLOCK_SIZE)
            struct.pack_into('<H', block, 0, VLP16_BLOCK_FLAG)
            struct.pack_into('<H', block, 2, blk_az % 36000)

            # Fill channels. For duplicate channels, keep the closest return.
            # Channels 0-15 = first firing, 16-31 = second firing (same data
            # duplicated for single-return mode).
            for i in range(len(blk_ch)):
                ch = int(blk_ch[i])
                offset = 4 + ch * VLP16_CHANNEL_DATA_SIZE
                # Check if already filled - keep closer point
                existing_dist = struct.unpack_from('<H', block, offset)[0]
                new_dist = int(blk_dist[i])
                if existing_dist == 0 or new_dist < existing_dist:
                    struct.pack_into('<H', block, offset, new_dist)
                    block[offset + 2] = int(blk_refl[i])

                # Mirror to second firing sequence (channels 16-31)
                offset2 = 4 + (ch + VLP16_LASERS) * VLP16_CHANNEL_DATA_SIZE
                existing_dist2 = struct.unpack_from('<H', block, offset2)[0]
                if existing_dist2 == 0 or new_dist < existing_dist2:
                    struct.pack_into('<H', block, offset2, new_dist)
                    block[offset2 + 2] = int(blk_refl[i])

            blocks.append(bytes(block))
            self._points_sent += int(mask.sum())

        # Pack blocks into packets (12 blocks per packet)
        packets = []
        n_blocks = len(blocks)

        for pkt_start in range(0, n_blocks, VLP16_BLOCKS_PER_PACKET):
            pkt_end = min(pkt_start + VLP16_BLOCKS_PER_PACKET, n_blocks)
            pkt = bytearray(VLP16_PACKET_SIZE)

            # Fill data blocks
            for i in range(pkt_end - pkt_start):
                offset = i * VLP16_BLOCK_SIZE
                pkt[offset:offset + VLP16_BLOCK_SIZE] = blocks[pkt_start + i]

            # Fill remaining blocks with empty blocks (continue azimuth)
            for i in range(pkt_end - pkt_start, VLP16_BLOCKS_PER_PACKET):
                offset = i * VLP16_BLOCK_SIZE
                # Use last valid azimuth + step, or 0 if no blocks
                if pkt_end > pkt_start:
                    last_az = struct.unpack_from(
                        '<H', blocks[pkt_end - 1], 2)[0]
                    fill_az = (last_az + AZIMUTH_STEP_CDEG *
                               (i - (pkt_end - pkt_start) + 1)) % 36000
                else:
                    fill_az = 0
                empty = _build_empty_block(fill_az)
                pkt[offset:offset + VLP16_BLOCK_SIZE] = empty

            # Timestamp (microseconds since top of hour)
            pkt_timestamp = (base_timestamp_us +
                             pkt_start * 55) & 0xFFFFFFFF  # ~55us per block
            struct.pack_into('<I', pkt, VLP16_DATA_BLOCK_TOTAL, pkt_timestamp)

            # Factory bytes
            pkt[VLP16_DATA_BLOCK_TOTAL + 4] = return_mode & 0xFF
            pkt[VLP16_DATA_BLOCK_TOTAL + 5] = VLP16_PRODUCT_ID

            packets.append(bytes(pkt))

        return packets

    # -- Stats override --

    def _stats_callback(self):
        self.get_logger().info(
            f'Status: frames={self._frame_count}, '
            f'points_sent={self._points_sent}, '
            f'device_ip={self.device_ip}')

    # -- Cleanup --

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelodyneEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
