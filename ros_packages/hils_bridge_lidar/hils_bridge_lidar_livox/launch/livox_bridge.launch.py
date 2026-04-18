"""Launch file for Livox Mid360 HILS bridge node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='USB CDC serial port to W5500-EVB-Pico2'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/livox/lidar',
                              description='PointCloud2 topic from simulation'),
        DeclareLaunchArgument('imu_topic', default_value='/livox/imu',
                              description='IMU topic from simulation'),
        DeclareLaunchArgument('enable_imu', default_value='true',
                              description='Enable IMU data forwarding'),
        DeclareLaunchArgument('max_hz', default_value='10.0',
                              description='Max point cloud publish rate'),
        DeclareLaunchArgument('max_points_per_frame', default_value='4600',
                              description='Max points per HILS frame'),
        DeclareLaunchArgument('point_data_type', default_value='1',
                              description='1=CartesianHigh(14B), 2=CartesianLow(8B)'),
        DeclareLaunchArgument('lidar_ip', default_value='192.168.1.12',
                              description='Emulated LiDAR IP address'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.5',
                              description='Target host IP address'),
        DeclareLaunchArgument('serial_number', default_value='0TFDFH600100511',
                              description='Emulated LiDAR serial number'),

        Node(
            package='hils_bridge_lidar_livox',
            executable='livox_bridge_node',
            name='hils_livox_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'enable_imu': LaunchConfiguration('enable_imu'),
                'max_hz': LaunchConfiguration('max_hz'),
                'max_points_per_frame': LaunchConfiguration('max_points_per_frame'),
                'point_data_type': LaunchConfiguration('point_data_type'),
                'lidar_ip': LaunchConfiguration('lidar_ip'),
                'host_ip': LaunchConfiguration('host_ip'),
                'serial_number': LaunchConfiguration('serial_number'),
            }],
            output='screen',
        ),
    ])
