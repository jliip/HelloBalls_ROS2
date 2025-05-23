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
        # Change this to match your camera topic (from cv_publisher)
        default_value='/cv_publisher/image_raw',
        description='Topic to subscribe for input images'
    )

    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/yolo/detections',
        description='Topic to publish detection images'
    )

    control_topic_arg = DeclareLaunchArgument(
        'control_topic',
        default_value='/cmd_vel',
        description='Topic to publish control commands'
    )

    # Create node
    detector_node = Node(
        package='yolo11_detector',
        executable='yolo_detector',
        name='yolo_detector_node',
        parameters=[{
            'show_window': LaunchConfiguration('show_window'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'control_topic': LaunchConfiguration('control_topic')
        }],
        output='screen'
    )

    return LaunchDescription([
        show_window_arg,
        input_topic_arg,
        output_topic_arg,
        control_topic_arg,
        detector_node
    ])