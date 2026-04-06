from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('jpeg_quality', default_value='50'),
        DeclareLaunchArgument('image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('max_fps', default_value='15.0'),
        DeclareLaunchArgument('frame_width', default_value='640'),
        DeclareLaunchArgument('frame_height', default_value='480'),

        Node(
            package='hils_bridge_camera_uvc',
            executable='uvc_bridge_node',
            name='hils_uvc_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                'image_topic': LaunchConfiguration('image_topic'),
                'max_fps': LaunchConfiguration('max_fps'),
                'frame_width': LaunchConfiguration('frame_width'),
                'frame_height': LaunchConfiguration('frame_height'),
            }],
            output='screen',
        ),
    ])
