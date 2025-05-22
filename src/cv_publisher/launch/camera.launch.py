from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cv_publisher',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen'
        ),
        Node(
            package='cv_subscriber',
            executable='image_subscriber',
            name='image_subscriber',
            output='screen'
        )
    ])