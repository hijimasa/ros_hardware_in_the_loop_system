#!/usr/bin/env python3
"""
HILS GPS NMEA Serial Bridge Node

Subscribes to a ROS NavSatFix topic from a simulator and writes NMEA 0183
sentences ($GPGGA and $GPRMC) to a serial port (FT234X USB-serial adapter
with cross-connection to the robot PC).

Optionally subscribes to a TwistStamped topic for ground speed and course
in the $GPRMC sentence.

Data flow:
    Simulation (NavSatFix) -> this node -> NMEA serial -> FT234X -> Robot PC
"""

import math
from datetime import datetime, timezone

import rclpy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from geometry_msgs.msg import TwistStamped

from hils_bridge_base.serial_bridge_base import SerialBridgeBase


def _nmea_checksum(sentence: str) -> str:
    """Compute NMEA checksum: XOR of all characters between '$' and '*'.

    Args:
        sentence: The NMEA sentence content (without '$' prefix and '*XX' suffix).

    Returns:
        Two-character uppercase hex checksum string.
    """
    cs = 0
    for ch in sentence:
        cs ^= ord(ch)
    return f'{cs:02X}'


def _deg_to_nmea_lat(deg: float) -> tuple:
    """Convert decimal degrees latitude to NMEA format (ddmm.mmmm, N/S).

    Args:
        deg: Latitude in decimal degrees (positive=N, negative=S).

    Returns:
        Tuple of (formatted string "ddmm.mmmm", hemisphere "N" or "S").
    """
    hemisphere = 'N' if deg >= 0 else 'S'
    deg = abs(deg)
    d = int(deg)
    m = (deg - d) * 60.0
    return f'{d:02d}{m:07.4f}', hemisphere


def _deg_to_nmea_lon(deg: float) -> tuple:
    """Convert decimal degrees longitude to NMEA format (dddmm.mmmm, E/W).

    Args:
        deg: Longitude in decimal degrees (positive=E, negative=W).

    Returns:
        Tuple of (formatted string "dddmm.mmmm", hemisphere "E" or "W").
    """
    hemisphere = 'E' if deg >= 0 else 'W'
    deg = abs(deg)
    d = int(deg)
    m = (deg - d) * 60.0
    return f'{d:03d}{m:07.4f}', hemisphere


def _stamp_to_utc(stamp) -> datetime:
    """Convert a ROS Time stamp to a UTC datetime object.

    Args:
        stamp: ROS builtin_interfaces/Time with sec and nanosec fields.

    Returns:
        datetime object in UTC timezone.
    """
    epoch_sec = stamp.sec + stamp.nanosec * 1e-9
    return datetime.fromtimestamp(epoch_sec, tz=timezone.utc)


class GpsBridgeNode(SerialBridgeBase):
    """ROS2 node that converts NavSatFix messages to NMEA serial output."""

    def __init__(self):
        super().__init__(
            node_name='hils_gps_bridge',
            default_baudrate=9600,
        )

        # Additional parameters
        self.declare_parameter('fix_topic', '/gps/fix')
        self.declare_parameter('vel_topic', '/gps/vel')
        self.declare_parameter('enable_velocity', True)

        # Latest velocity data (protected by the base class serial lock is not
        # needed here since ROS callbacks are single-threaded by default)
        self._latest_vel = None

        # Subscribe to NavSatFix
        fix_topic = self.get_parameter('fix_topic').value
        self.create_subscription(
            NavSatFix, fix_topic, self._fix_callback, 1)

        # Optionally subscribe to TwistStamped for velocity
        if self.get_parameter('enable_velocity').value:
            vel_topic = self.get_parameter('vel_topic').value
            self.create_subscription(
                TwistStamped, vel_topic, self._vel_callback, 1)
            self.get_logger().info(f'Velocity subscription: {vel_topic}')

        self.get_logger().info(
            f'GPS NMEA Bridge started: fix_topic={fix_topic}, '
            f'baudrate={self.get_parameter("baudrate").value}, '
            f'max_hz={self.get_parameter("max_hz").value}')

    def _vel_callback(self, msg: TwistStamped):
        """Cache the latest velocity message for use in GPRMC."""
        self._latest_vel = msg

    def _fix_callback(self, msg: NavSatFix):
        """Convert NavSatFix to NMEA sentences and write to serial."""
        if not self.check_rate_limit():
            return

        # Skip invalid fixes
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            return

        # Convert timestamp to UTC
        utc = _stamp_to_utc(msg.header.stamp)
        time_str = utc.strftime('%H%M%S') + f'.{utc.microsecond // 10000:02d}'
        date_str = utc.strftime('%d%m%y')

        # Convert coordinates
        lat_str, lat_hem = _deg_to_nmea_lat(msg.latitude)
        lon_str, lon_hem = _deg_to_nmea_lon(msg.longitude)

        # Altitude (above mean sea level)
        alt = msg.altitude

        # Determine fix quality for GGA
        # STATUS_FIX=0, STATUS_SBAS_FIX=1, STATUS_GBAS_FIX=2
        if msg.status.status == NavSatStatus.STATUS_SBAS_FIX:
            fix_quality = 2  # DGPS
        elif msg.status.status == NavSatStatus.STATUS_GBAS_FIX:
            fix_quality = 2  # DGPS
        else:
            fix_quality = 1  # Standard GPS

        # Build GPGGA sentence
        gga = self._build_gpgga(
            time_str, lat_str, lat_hem, lon_str, lon_hem,
            fix_quality, alt)

        # Compute speed and course from velocity if available
        speed_knots = ''
        course_deg = ''
        if self._latest_vel is not None:
            vx = self._latest_vel.twist.linear.x
            vy = self._latest_vel.twist.linear.y
            speed_ms = math.sqrt(vx * vx + vy * vy)
            speed_knots = f'{speed_ms * 1.943844:.1f}'
            if speed_ms > 0.1:
                course = math.degrees(math.atan2(vy, vx))
                # Convert from math convention (CCW from east) to
                # navigation convention (CW from north)
                course = 90.0 - course
                if course < 0:
                    course += 360.0
                if course >= 360.0:
                    course -= 360.0
                course_deg = f'{course:.1f}'

        # Build GPRMC sentence
        rmc = self._build_gprmc(
            time_str, lat_str, lat_hem, lon_str, lon_hem,
            speed_knots, course_deg, date_str)

        # Write both sentences to serial
        output = (gga + rmc).encode('ascii')
        self.serial_write(output)

    def _build_gpgga(self, time_str: str, lat: str, lat_hem: str,
                     lon: str, lon_hem: str, fix_quality: int,
                     altitude: float) -> str:
        """Build a $GPGGA NMEA sentence.

        Args:
            time_str: UTC time as hhmmss.ss
            lat: Latitude as ddmm.mmmm
            lat_hem: N or S
            lon: Longitude as dddmm.mmmm
            lon_hem: E or W
            fix_quality: GPS fix quality (0=invalid, 1=GPS, 2=DGPS)
            altitude: Altitude above MSL in meters

        Returns:
            Complete NMEA sentence with checksum and CRLF terminator.
        """
        # Fields: time, lat, N/S, lon, E/W, quality, num_sats, HDOP,
        #         altitude, M, geoidal_sep, M, dgps_age, dgps_station
        body = (f'GPGGA,{time_str},{lat},{lat_hem},{lon},{lon_hem},'
                f'{fix_quality},08,1.0,{altitude:.1f},M,0.0,M,,')
        cs = _nmea_checksum(body)
        return f'${body}*{cs}\r\n'

    def _build_gprmc(self, time_str: str, lat: str, lat_hem: str,
                     lon: str, lon_hem: str, speed_knots: str,
                     course_deg: str, date_str: str) -> str:
        """Build a $GPRMC NMEA sentence.

        Args:
            time_str: UTC time as hhmmss.ss
            lat: Latitude as ddmm.mmmm
            lat_hem: N or S
            lon: Longitude as dddmm.mmmm
            lon_hem: E or W
            speed_knots: Speed over ground in knots (empty if unknown)
            course_deg: Course over ground in degrees (empty if unknown)
            date_str: UTC date as ddmmyy

        Returns:
            Complete NMEA sentence with checksum and CRLF terminator.
        """
        # Fields: time, status, lat, N/S, lon, E/W, speed, course,
        #         date, mag_var, mag_var_dir, mode
        body = (f'GPRMC,{time_str},A,{lat},{lat_hem},{lon},{lon_hem},'
                f'{speed_knots},{course_deg},{date_str},,,A')
        cs = _nmea_checksum(body)
        return f'${body}*{cs}\r\n'


def main(args=None):
    rclpy.init(args=args)
    node = GpsBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
