from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Launch arguments
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        description='Whether to show detection window'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/image_raw',
        description='Topic to subscribe for input images'
    )

    # Start the camera publisher node with the correct executable name
    camera_node = Node(
        package='cv_publisher',
        executable='camera_publisher',  # Using the correct executable name
        name='camera_publisher_node',
        output='screen'
    )

    # Start the YOLO detector node
    detector_node = Node(
        package='yolo11_detector',
        executable='yolo_detector',
        name='yolo_detector_node',
        parameters=[{
            'show_window': LaunchConfiguration('show_window'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': '/yolo/detections',
            'control_topic': '/cmd_vel'
        }],
        output='screen'
    )

    return LaunchDescription([
        show_window_arg,
        input_topic_arg,
        camera_node,
        detector_node
    ])