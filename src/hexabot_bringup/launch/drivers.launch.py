"""Payload and sensor drivers of the real robot.

Hexabot drivers (always available):
  laser rangefinder, IR illuminator, thermal camera (hexabot_perception)
Vendor drivers (install them first, see docs/rk3588_setup.md); each one is
started only when its package is found:
  ldlidar_stl_ros2   LD19 / STL-19P lidar        -> /scan
  orbbec_camera      Gemini 336 depth camera     -> /depth_camera/*, IMU
  depthai_ros_driver OAK-D Pro W (alternative)
  usb_cam            low-light aiming camera     -> /lowlight_camera/*
  imu_filter_madgwick  camera IMU -> /imu/data
  nmea_navsat_driver   GNSS                      -> /navsat
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def have(pkg):
    try:
        get_package_share_directory(pkg)
        return True
    except PackageNotFoundError:
        return False


def launch_setup(context):
    cfg = os.path.join(get_package_share_directory('hexabot_bringup'), 'config', 'drivers.yaml')
    out = [
        Node(package='hexabot_lrf', executable='lrf_node', name='lrf', parameters=[cfg], output='screen'),
        Node(package='hexabot_illuminator', executable='illuminator_node', name='illuminator',
             parameters=[cfg], output='screen'),
    ]
    if have('hexabot_perception'):
        out.append(Node(package='hexabot_perception', executable='thermal_camera', name='thermal_camera',
                        parameters=[cfg], output='screen'))
    if have('ldlidar_stl_ros2'):
        out.append(Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node', name='lidar',
                        parameters=[{'product_name': 'LDLiDAR_LD19', 'topic_name': 'scan',
                                     'frame_id': 'laser_link', 'port_name': '/dev/hexabot/lidar',
                                     'port_baudrate': 230400, 'laser_scan_dir': True,
                                     'enable_angle_crop_func': True,
                                     # The head and the GNSS pad sit behind the lidar.
                                     'angle_crop_min': 160.0, 'angle_crop_max': 200.0}]))
    else:
        out.append(LogInfo(msg='ldlidar_stl_ros2 not installed: no /scan'))
    if have('orbbec_camera'):
        out.append(Node(package='orbbec_camera', executable='orbbec_camera_node', name='depth_camera',
                        namespace='depth_camera',
                        parameters=[{'camera_name': 'depth_camera', 'enable_point_cloud': True,
                                     'enable_accel': True, 'enable_gyro': True,
                                     'enable_sync_output_accel_gyro': True,
                                     'depth_width': 640, 'depth_height': 400, 'depth_fps': 15,
                                     'color_width': 640, 'color_height': 480, 'color_fps': 15}]))
    if have('usb_cam'):
        out.append(Node(package='usb_cam', executable='usb_cam_node_exe', name='lowlight_camera',
                        namespace='lowlight_camera',
                        parameters=[{'video_device': '/dev/hexabot/lowlight', 'framerate': 25.0,
                                     'image_width': 1280, 'image_height': 720, 'pixel_format': 'mjpeg2rgb',
                                     'frame_id': 'lowlight_camera_link_optical_frame'}]))
    if have('imu_filter_madgwick'):
        out.append(Node(package='imu_filter_madgwick', executable='imu_filter_madgwick_node', name='imu_filter',
                        parameters=[{'use_mag': False, 'publish_tf': False, 'world_frame': 'enu'}],
                        remappings=[('imu/data_raw', '/depth_camera/gyro_accel/sample'),
                                    ('imu/data', '/imu/data')]))
    if have('nmea_navsat_driver'):
        out.append(Node(package='nmea_navsat_driver', executable='nmea_serial_driver', name='gnss',
                        parameters=[{'port': '/dev/hexabot/gnss', 'baud': 38400, 'frame_id': 'gnss_link'}],
                        remappings=[('fix', '/navsat')]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('unused', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
