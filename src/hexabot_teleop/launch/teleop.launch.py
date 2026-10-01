"""Gamepad teleoperation: joy_node + hexabot teleop."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('hexabot_teleop'), 'config', 'gamepad.yaml')
    sim = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joy_device', default_value='0'),
        Node(package='joy', executable='joy_node', name='joy',
             parameters=[{'device_id': LaunchConfiguration('joy_device'), 'autorepeat_rate': 20.0,
                          'deadzone': 0.05, 'use_sim_time': sim}]),
        Node(package='hexabot_teleop', executable='teleop_node', name='teleop',
             parameters=[cfg, {'use_sim_time': sim}]),
    ])
