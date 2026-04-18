"""Launch file for PWM servo + encoder bridge node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('hils_bridge_actuator_pwm')
    default_params_file = os.path.join(pkg_dir, 'config', 'default_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
                              description='Serial port for RP2040 USB CDC'),
        DeclareLaunchArgument('baudrate', default_value='115200',
                              description='Serial baudrate'),
        DeclareLaunchArgument('max_hz', default_value='50.0',
                              description='Maximum servo update rate'),
        DeclareLaunchArgument('joint_state_topic', default_value='/joint_states',
                              description='JointState topic from simulation'),
        DeclareLaunchArgument('servo_channels', default_value='4',
                              description='Number of servo channels (1-4)'),
        DeclareLaunchArgument('encoder_cpr', default_value='1000',
                              description='Encoder counts per revolution'),
        DeclareLaunchArgument('servo_min_angle', default_value='-1.5708',
                              description='Minimum servo angle (rad)'),
        DeclareLaunchArgument('servo_max_angle', default_value='1.5708',
                              description='Maximum servo angle (rad)'),
        DeclareLaunchArgument('servo_min_pulse_us', default_value='500',
                              description='Minimum servo pulse width (us)'),
        DeclareLaunchArgument('servo_max_pulse_us', default_value='2500',
                              description='Maximum servo pulse width (us)'),

        Node(
            package='hils_bridge_actuator_pwm',
            executable='pwm_bridge_node',
            name='hils_pwm_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'max_hz': LaunchConfiguration('max_hz'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'servo_channels': LaunchConfiguration('servo_channels'),
                'encoder_cpr': LaunchConfiguration('encoder_cpr'),
                'servo_min_angle': LaunchConfiguration('servo_min_angle'),
                'servo_max_angle': LaunchConfiguration('servo_max_angle'),
                'servo_min_pulse_us': LaunchConfiguration('servo_min_pulse_us'),
                'servo_max_pulse_us': LaunchConfiguration('servo_max_pulse_us'),
            }],
            output='screen',
        ),
    ])
