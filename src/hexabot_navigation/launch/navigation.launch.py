"""Mapping and autonomous navigation.

    ros2 launch hexabot_navigation navigation.launch.py [use_sim_time:=true] [slam:=true]

EKF (leg odometry + IMU) -> odom; slam_toolbox -> map; Nav2 (MPPI omni).
With slam:=false a saved map is used (map:=/path/to/map.yaml, map_server + AMCL).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NAV_NODES = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
             'bt_navigator', 'waypoint_follower', 'velocity_smoother']


def launch_setup(context):
    share = get_package_share_directory('hexabot_navigation')
    sim = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    slam = LaunchConfiguration('slam').perform(context) == 'true'
    params = os.path.join(share, 'config', 'nav2_params.yaml')
    common = {'use_sim_time': sim}
    out = [
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
             parameters=[os.path.join(share, 'config', 'ekf.yaml'), common]),
        # The EKF owns odom -> base_footprint now.
        ExecuteProcess(cmd=['ros2', 'param', 'set', '/locomotion', 'publish_tf', 'false'], output='log'),
    ]
    if slam:
        out.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
                        output='screen', parameters=[os.path.join(share, 'config', 'slam_toolbox.yaml'), common]))
    else:
        out += [
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[{'yaml_filename': LaunchConfiguration('map').perform(context)}, common]),
            Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
                 parameters=[{'base_frame_id': 'base_footprint', 'robot_model_type': 'nav2_amcl::OmniMotionModel',
                              'scan_topic': '/scan', 'laser_max_range': 11.0}, common]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_localization', output='screen',
                 parameters=[{'autostart': True, 'node_names': ['map_server', 'amcl']}, common]),
        ]
    remap = [('cmd_vel', 'cmd_vel_nav')]
    out += [
        Node(package='nav2_controller', executable='controller_server', output='screen',
             parameters=[params, common], remappings=remap),
        Node(package='nav2_smoother', executable='smoother_server', name='smoother_server', output='screen',
             parameters=[params, common]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params, common]),
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server', output='screen',
             parameters=[params, common], remappings=remap),
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator', output='screen',
             parameters=[params, common]),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
             output='screen', parameters=[params, common]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             output='screen', parameters=[params, common],
             remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', '/cmd_vel')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
             output='screen', parameters=[{'autostart': True, 'node_names': NAV_NODES}, common]),
    ]
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('slam', default_value='true'),
        DeclareLaunchArgument('map', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
