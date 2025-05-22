#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

class ESP32MotorController(Node):
    def __init__(self):
        super().__init__('esp32_motor_controller')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyS1',
                              ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                description='Serial port for ESP32 communication'))
        self.declare_parameter('baud_rate', 115200,
                              ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                                description='Baud rate for serial communication'))
        self.declare_parameter('motor_max_speed', 1000,
                              ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                                                description='Maximum motor speed value'))
        self.declare_parameter('parity', 'odd',
                              ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                description='Parity check: none, even, odd, mark, space'))
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.motor_max_speed = self.get_parameter('motor_max_speed').value
        self.parity_param = self.get_parameter('parity').value
        
        # Map parity parameter to serial.PARITY_* constants
        parity_map = {
            'none': serial.PARITY_NONE,
            'even': serial.PARITY_EVEN,
            'odd': serial.PARITY_ODD,
            'mark': serial.PARITY_MARK,
            'space': serial.PARITY_SPACE
        }
        self.parity = parity_map.get(self.parity_param.lower(), serial.PARITY_ODD)
        
        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        self.direct_cmd_sub = self.create_subscription(
            String,
            'motor/direct_command',
            self.direct_cmd_callback,
            10)
            
        # Status publisher
        self.status_pub = self.create_publisher(String, 'motor/status', 10)
        
        # Regular status update timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f"ESP32 Motor Controller initialized with {self.serial_port} at {self.baud_rate} baud")
        self.get_logger().info(f"Using parity: {self.parity_param}")

    def connect_serial(self):
        """Try to connect to serial port, with retry logic"""
        while rclpy.ok():
            try:
                if self.serial_conn:
                    self.serial_conn.close()
                    
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    parity=self.parity,
                    timeout=1.0
                )
                self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
                return True
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port: {e}")
                self.get_logger().info("Retrying in 5 seconds...")
                time.sleep(5.0)
        return False

    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor commands"""
        # Basic differential drive conversion
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Simple conversion for differential drive
        # Adjust these calculations based on your robot's kinematics
        left_speed = int((linear_x - angular_z) * self.motor_max_speed)
        right_speed = int((linear_x + angular_z) * self.motor_max_speed)
        
        # Clamp values
        left_speed = max(min(left_speed, self.motor_max_speed), -self.motor_max_speed)
        right_speed = max(min(right_speed, self.motor_max_speed), -self.motor_max_speed)
        
        # Send command to ESP32
        self.send_motor_command(1, left_speed, right_speed)
        
    def direct_cmd_callback(self, msg):
        """Handle direct command strings"""
        try:
            # Parse commands like "status,motor1_speed,motor2_speed"
            command = msg.data.strip()
            self.get_logger().info(f"Received direct command: {command}")
            
            # Send the command directly to ESP32
            if self.serial_conn and self.serial_conn.is_open:
                command_bytes = (command + '\n').encode('utf-8')
                self.serial_conn.write(command_bytes)
                self.get_logger().debug(f"Sent: {command}")
            else:
                self.get_logger().error("Cannot send command: Serial port not open")
                self.connect_serial()
        except Exception as e:
            self.get_logger().error(f"Error processing direct command: {e}")

    def send_motor_command(self, status, motor1_speed, motor2_speed):
        """Send motor command to ESP32"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warning("Serial connection not available, attempting to reconnect...")
            self.connect_serial()
            return
            
        try:
            # Create command string in format "status,motor1_speed,motor2_speed"
            command = f"{status},{motor1_speed},{motor2_speed}\n"
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().debug(f"Sent motor command: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending motor command: {e}")
            self.connect_serial()

    def publish_status(self):
        """Publish controller status"""
        status_msg = String()
        if self.serial_conn and self.serial_conn.is_open:
            status_msg.data = "Connected to ESP32"
        else:
            status_msg.data = "Disconnected from ESP32"
            # Try to reconnect
            self.connect_serial()
        
        self.status_pub.publish(status_msg)

    def __del__(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()