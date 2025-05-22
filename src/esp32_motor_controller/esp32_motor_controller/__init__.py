from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

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
        
    parity_arg = DeclareLaunchArgument(
        'parity',
        default_value='odd',
        description='Parity setting for serial communication')
        
    motor_max_speed_arg = DeclareLaunchArgument(
        'motor_max_speed',
        default_value='1000',
        description='Maximum motor speed value')

    # Motor controller node
    motor_controller_node = Node(
        package='esp32_motor_controller',
        executable='motor_controller_node',
        name='esp32_motor_controller',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'parity': LaunchConfiguration('parity'),
            'motor_max_speed': LaunchConfiguration('motor_max_speed'),
        }],
        output='screen'
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        parity_arg,
        motor_max_speed_arg,
        motor_controller_node
    ])