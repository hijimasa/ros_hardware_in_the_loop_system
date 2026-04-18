"""Launch file for IMU Witmotion WT901 serial bridge node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hils_bridge_serial_imu')
    default_params_file = os.path.join(pkg_dir, 'config', 'default_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='Serial port for FT234X adapter'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='Serial baudrate (WT901 default)'),
        DeclareLaunchArgument('max_hz', default_value='50.0',
                              description='Maximum output rate'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data',
                              description='Imu topic from simulation'),

        Node(
            package='hils_bridge_serial_imu',
            executable='imu_bridge_node',
            name='hils_imu_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'max_hz': LaunchConfiguration('max_hz'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            }],
            output='screen',
        ),
    ])
