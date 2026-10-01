"""Show the robot model in RViz with joint sliders.

    ros2 launch hexabot_description display.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('hexabot_description')
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([share, 'urdf', 'hexabot.urdf.xacro']),
                 ' use_mock_hardware:=true']),
        value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', PathJoinSubstitution([share, 'rviz', 'hexabot.rviz'])],
             condition=None),
    ])
