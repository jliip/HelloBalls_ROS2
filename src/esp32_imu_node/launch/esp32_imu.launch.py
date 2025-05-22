#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('esp32_imu_node'), 'config')
    config_file = os.path.join(config_dir, 'esp32_imu_params.yaml')
    
    # Declare arguments
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

    # IMU Node
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
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        frame_id_arg,
        esp32_imu_node
    ])