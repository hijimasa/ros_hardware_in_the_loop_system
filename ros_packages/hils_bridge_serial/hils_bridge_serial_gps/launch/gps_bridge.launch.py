"""Launch file for GPS NMEA serial bridge node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hils_bridge_serial_gps')
    default_params_file = os.path.join(pkg_dir, 'config', 'default_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='Serial port for FT234X adapter'),
        DeclareLaunchArgument('baudrate', default_value='9600',
                              description='Serial baudrate (standard GPS NMEA)'),
        DeclareLaunchArgument('max_hz', default_value='10.0',
                              description='Maximum NMEA output rate'),
        DeclareLaunchArgument('fix_topic', default_value='/gps/fix',
                              description='NavSatFix topic from simulation'),
        DeclareLaunchArgument('vel_topic', default_value='/gps/vel',
                              description='TwistStamped topic for velocity'),
        DeclareLaunchArgument('enable_velocity', default_value='true',
                              description='Subscribe to velocity topic for GPRMC speed/course'),

        Node(
            package='hils_bridge_serial_gps',
            executable='gps_bridge_node',
            name='hils_gps_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'max_hz': LaunchConfiguration('max_hz'),
                'fix_topic': LaunchConfiguration('fix_topic'),
                'vel_topic': LaunchConfiguration('vel_topic'),
                'enable_velocity': LaunchConfiguration('enable_velocity'),
            }],
            output='screen',
        ),
    ])
