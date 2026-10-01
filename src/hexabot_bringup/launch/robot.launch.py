"""Hexabot core: description, ros2_control, controllers and locomotion.

    ros2 launch hexabot_bringup robot.launch.py                     # real robot
    ros2 launch hexabot_bringup robot.launch.py hardware:=simbus    # no hardware
    ros2 launch hexabot_bringup robot.launch.py hardware:=mock

hardware: real   STS3215 bus on servo_port
          simbus the same driver on an in-memory servo bus
          mock   ros2_control mock components (no driver code at all)
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    hw = LaunchConfiguration('hardware').perform(context)
    if hw not in ('real', 'simbus', 'mock'):
        raise RuntimeError(f'hardware must be real, simbus or mock (got {hw})')
    sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    desc = get_package_share_directory('hexabot_description')
    bringup = get_package_share_directory('hexabot_bringup')
    robot_description = xacro.process_file(
        os.path.join(desc, 'urdf', 'hexabot.urdf.xacro'),
        mappings={
            'use_mock_hardware': 'true' if hw == 'mock' else 'false',
            'simulated_bus': 'true' if hw == 'simbus' else 'false',
            'servo_port': LaunchConfiguration('servo_port').perform(context),
        }).toxml()
    controllers = os.path.join(bringup, 'config', 'controllers.yaml')
    geometry = os.path.join(desc, 'config', 'geometry.yaml')
    locomotion = os.path.join(bringup, 'config', 'locomotion.yaml')

    # One spawner for all controllers (parallel spawners contend for a lock file).
    spawner = Node(package='controller_manager', executable='spawner', output='screen',
                   arguments=['joint_state_broadcaster', 'leg_controller', 'head_controller',
                              '--controller-manager', '/controller_manager',
                              '--controller-manager-timeout', '30'],
                   parameters=[{'use_sim_time': sim_time}])

    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description, 'use_sim_time': sim_time}]),
        Node(package='controller_manager', executable='ros2_control_node', output='screen',
             parameters=[controllers, {'use_sim_time': sim_time}],
             remappings=[('~/robot_description', '/robot_description')]),
        spawner,
        Node(package='hexabot_locomotion', executable='locomotion_node', output='screen',
             parameters=[geometry, locomotion, {
                 'start_mode': LaunchConfiguration('start_mode').perform(context),
                 'use_sim_time': sim_time,
             }],
             remappings=[('imu', LaunchConfiguration('imu_topic').perform(context))]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='real', description='real | simbus | mock'),
        DeclareLaunchArgument('servo_port', default_value='/dev/hexabot/servos'),
        DeclareLaunchArgument('start_mode', default_value='wait', description='wait | stand'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
