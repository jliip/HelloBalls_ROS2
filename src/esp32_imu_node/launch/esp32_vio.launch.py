#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os

def generate_launch_description():
    # Get configuration file paths
    config_dir = os.path.join(get_package_share_directory('esp32_imu_node'), 'config')
    esp32_config_file = os.path.join(config_dir, 'esp32_imu_params.yaml')
    
    vio_config_file = os.path.join(
        get_package_prefix('hobot_vio'),
        "lib/hobot_vio/config/realsenseD435i_color.yaml")
    print("VIO config_file_path is", vio_config_file)

    # Declare arguments for ESP32 IMU node
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS1',
        description='Serial port for ESP32 IMU'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages'
    )

    # USB Camera argument
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Path to camera device'
    )

    # ESP32 IMU Node
    esp32_imu_node = Node(
        package='esp32_imu_node',
        executable='esp32_imu_node',
        name='esp32_imu_node',
        output='screen',
        parameters=[
            {'serial_port': LaunchConfiguration('serial_port')},
            {'baud_rate': LaunchConfiguration('baud_rate')},
            {'frame_id': LaunchConfiguration('frame_id')}
        ]
    )

    # USB camera node
    usb_cam_node = ExecuteProcess(
        cmd=['ros2', 'launch', 'usb_cam', 'demo_launch.py',
             'video_device:=' + LaunchConfiguration('camera_device'),
             'image_width:=640',
             'image_height:=480',
             'framerate:=30']
    )

    # Horizon VIO node
    horizon_vio_node = Node(
        package='hobot_vio',
        executable='hobot_vio',
        output='screen',
        parameters=[
            {"path_config": vio_config_file},
            {"image_topic": "/image_raw"},  # USB camera topic
            {"imu_topic": "/imu/data"},     # ESP32 IMU topic
            {"sample_gap": 2}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # CPU boost command for better performance
    boost = ExecuteProcess(
        cmd=[[
            'echo 1 > /sys/devices/system/cpu/cpufreq/boost '
        ]],
        shell=True
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        frame_id_arg,
        camera_device_arg,
        esp32_imu_node,
        usb_cam_node,
        horizon_vio_node,
        boost
    ])