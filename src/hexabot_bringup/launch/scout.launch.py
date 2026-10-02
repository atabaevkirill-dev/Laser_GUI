"""Everything for a scouting run on the real robot.

    ros2 launch hexabot_bringup scout.launch.py [nav:=true] [teleop:=true]

robot core + drivers + perception (detection, tracking, thermal, analytics,
targeting) + terrain + navigation (SLAM, EKF, Nav2) + behaviours + foxglove.
Components whose packages are missing are skipped with a message.
"""
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(pkg, name, args=None):
    try:
        share = get_package_share_directory(pkg)
    except PackageNotFoundError:
        return LogInfo(msg=f'{pkg} not installed: {name} skipped')
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(f'{share}/launch/{name}'),
                                    launch_arguments=(args or {}).items())


def launch_setup(context):
    on = lambda name: LaunchConfiguration(name).perform(context) == 'true'  # noqa: E731
    out = [
        include('hexabot_bringup', 'robot.launch.py', {'hardware': LaunchConfiguration('hardware').perform(context),
                                                        'start_mode': 'wait'}),
        include('hexabot_bringup', 'drivers.launch.py'),
        include('hexabot_perception', 'perception.launch.py'),
        include('hexabot_terrain', 'terrain.launch.py'),
    ]
    if on('nav'):
        out.append(include('hexabot_navigation', 'navigation.launch.py'))
    if on('behaviors'):
        out.append(include('hexabot_behaviors', 'behaviors.launch.py'))
    if on('teleop'):
        out.append(include('hexabot_teleop', 'teleop.launch.py'))
    try:
        get_package_share_directory('foxglove_bridge')
        out.append(Node(package='foxglove_bridge', executable='foxglove_bridge', name='foxglove_bridge',
                        parameters=[{'port': 8765, 'send_buffer_limit': 50000000}]))
    except PackageNotFoundError:
        out.append(LogInfo(msg='foxglove_bridge not installed: no web UI'))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware', default_value='real'),
        DeclareLaunchArgument('nav', default_value='true'),
        DeclareLaunchArgument('behaviors', default_value='true'),
        DeclareLaunchArgument('teleop', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
