"""Hexabot in Gazebo Harmonic on the scout test range.

    ros2 launch hexabot_gazebo sim.launch.py                 # day, GUI
    ros2 launch hexabot_gazebo sim.launch.py world:=night
    ros2 launch hexabot_gazebo sim.launch.py headless:=true

The same controllers, locomotion and device drivers run as on the robot; the
rangefinder and the illuminator use their simulation modes.
"""
import os

import xacro
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context) == 'true'
    gz = get_package_share_directory('hexabot_gazebo')
    desc = get_package_share_directory('hexabot_description')
    bringup = get_package_share_directory('hexabot_bringup')
    world_file = (os.path.join(gz, 'worlds', f'scout_range_{world}.sdf')
                  if world in ('day', 'night', 'physics') else world)
    controllers = os.path.join(bringup, 'config', 'controllers.yaml')
    robot_description = xacro.process_file(
        os.path.join(desc, 'urdf', 'hexabot.urdf.xacro'),
        mappings={'use_sim': 'true', 'controllers_file': controllers}).toxml()
    sim = {'use_sim_time': True}
    gz_args = f'-r -v 2 {world_file}' + (' -s --headless-rendering' if headless else '')

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'hexabot',
                            '-x', '0', '-y', '0', '-z', '0.115'])

    # One spawner for all controllers (parallel spawners contend for a lock file).
    spawner = Node(package='controller_manager', executable='spawner', output='screen',
                   arguments=['joint_state_broadcaster', 'leg_controller', 'head_controller',
                              '--controller-manager', '/controller_manager',
                              '--controller-manager-timeout', '60'], parameters=[sim])

    return [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH',
                               os.path.dirname(desc) + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', '')),
        # gz_ros2_control-system lives in the package's lib directory.
        SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',
                               os.path.join(get_package_prefix('gz_ros2_control'), 'lib') + os.pathsep
                               + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}, sim]),
        spawn,
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             parameters=[{'config_file': os.path.join(gz, 'config', 'bridge.yaml')}, sim]),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[spawner])),
        Node(package='hexabot_locomotion', executable='locomotion_node', output='screen',
             parameters=[os.path.join(desc, 'config', 'geometry.yaml'),
                         os.path.join(bringup, 'config', 'locomotion.yaml'),
                         {'start_mode': 'stand'}, sim],
             remappings=[('imu', '/imu/data')]),
        Node(package='hexabot_lrf', executable='lrf_node', name='lrf',
             parameters=[{'simulate': True, 'sim_scan_topic': '/lrf/scan'}, sim]),
        Node(package='hexabot_illuminator', executable='illuminator_node', name='illuminator',
             parameters=[{'transport': 'sim'}, sim]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='day',
                              description='day | night | physics (no rendering sensors) | path to .sdf'),
        DeclareLaunchArgument('headless', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])
