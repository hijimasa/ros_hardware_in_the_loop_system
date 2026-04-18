"""Launch file for pure software Velodyne VLP-16 emulator node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pointcloud_topic', default_value='/velodyne_points',
                              description='PointCloud2 topic from simulation'),
        DeclareLaunchArgument('max_hz', default_value='10.0',
                              description='Max point cloud publish rate'),
        DeclareLaunchArgument('max_points_per_frame', default_value='30000',
                              description='Max points per frame'),
        DeclareLaunchArgument('return_mode', default_value='55',
                              description='Return mode: 55=Strongest, 56=Last, 57=Dual'),
        DeclareLaunchArgument('downsample_mode', default_value='uniform',
                              description='Downsample: "uniform" or "near"'),
        DeclareLaunchArgument('device_ip', default_value='192.168.1.201',
                              description='IP assigned to the network interface (emulated VLP-16)'),
        DeclareLaunchArgument('host_ip', default_value='192.168.1.100',
                              description='Robot PC IP (velodyne_driver)'),
        DeclareLaunchArgument('network_interface', default_value='',
                              description='Network interface (e.g. "eth1"). Empty = auto from device_ip'),

        Node(
            package='hils_bridge_lidar_velodyne',
            executable='velodyne_emulator_node',
            name='hils_velodyne_emulator',
            parameters=[{
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'max_hz': LaunchConfiguration('max_hz'),
                'max_points_per_frame': LaunchConfiguration('max_points_per_frame'),
                'return_mode': LaunchConfiguration('return_mode'),
                'downsample_mode': LaunchConfiguration('downsample_mode'),
                'device_ip': LaunchConfiguration('device_ip'),
                'host_ip': LaunchConfiguration('host_ip'),
                'network_interface': LaunchConfiguration('network_interface'),
            }],
            output='screen',
        ),
    ])
