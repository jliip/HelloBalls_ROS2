#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyS1',
        description='Serial port for ESP32 communication')
        
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication')
        
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU messages')
        
    motor_max_speed_arg = DeclareLaunchArgument(
        'motor_max_speed',
        default_value='1000',
        description='Maximum motor speed value')
        
    enable_auto_test_arg = DeclareLaunchArgument(
        'enable_auto_test',
        default_value='True',
        description='Enable automatic motor test sequence')

    # Combined node
    combined_node = Node(
        package='esp32_combined_node',
        executable='esp32_combined_node',
        name='esp32_combined_node',
        output='screen',
        parameters=[
            {'serial_port': LaunchConfiguration('serial_port')},
            {'baud_rate': LaunchConfiguration('baud_rate')},
            {'frame_id': LaunchConfiguration('frame_id')},
            {'motor_max_speed': LaunchConfiguration('motor_max_speed')},
            {'enable_auto_test': LaunchConfiguration('enable_auto_test')}
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        frame_id_arg,
        motor_max_speed_arg,
        enable_auto_test_arg,
        combined_node
    ])