#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time
import json
import struct
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import String

class ESP32CombinedNode(Node):
    def __init__(self):
        super().__init__('esp32_combined_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyS1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('motor_max_speed', 1000)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('enable_auto_test', True)  # Enable auto motor test
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.motor_max_speed = self.get_parameter('motor_max_speed').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_auto_test = self.get_parameter('enable_auto_test').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()
        
        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.status_pub = self.create_publisher(String, 'motor/status', 10)
        
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
        
        # Timer for reading IMU data
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.read_imu_data)
        
        # Timer for publishing status
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Auto test variables and timer
        self.test_phase = 0
        self.test_counter = 0
        
        # Start auto test if enabled
        if self.enable_auto_test:
            # Match the IMU publish rate (50Hz) for smooth motion
            self.auto_test_timer = self.create_timer(1.0 / self.publish_rate, self.run_auto_test)
            self.get_logger().info(f"Auto motor test enabled - motors will move at {self.publish_rate}Hz")
        
        self.get_logger().info(f"ESP32 Combined Node initialized with {self.serial_port} at {self.baud_rate} baud")

    def connect_serial(self):
        """Try to connect to serial port, with retry logic"""
        while rclpy.ok():
            try:
                if self.serial_conn:
                    self.serial_conn.close()
                    
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baud_rate,
                    timeout=1.0
                )
                self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud")
                return True
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port: {e}")
                self.get_logger().info("Retrying in 5 seconds...")
                time.sleep(5.0)
        return False

    def run_auto_test(self):
        """Run automatic motor test sequence at 50Hz"""
        # Simple test sequence that will rotate through different movements
        test_sequences = [
            (1, 100, 100),    # Forward
            # (1, 0, 0),        # Stop
            # (1, -100, -100),  # Backward
            # (1, 0, 0),        # Stop
            # (1, -100, 100),   # Left spin
            # (1, 0, 0),        # Stop
            # (1, 100, -100),   # Right spin
            # (1, 0, 0),        # Stop
        ]
        
        # Only change commands every 50 cycles (about once per second)
        # This way we maintain 50Hz updates but change movement less frequently
        if self.test_phase % 50 == 0:
            movement_index = (self.test_phase // 50) % len(test_sequences)
            cmd = test_sequences[movement_index]
            
            # Log what we're doing (only when movement changes)
            action_names = ["Forward", "Stop", "Backward", "Stop", "Left spin", "Stop", "Right spin", "Stop"]
            self.get_logger().info(f"Auto test: {action_names[movement_index]} - sending {cmd}")
            
            # Send command
            self.send_motor_command(cmd[0], cmd[1], cmd[2])
            
            # Log test progress (complete cycle is 8 movements)
            if movement_index == 0 and self.test_phase > 0:
                self.test_counter += 1
                self.get_logger().info(f"Completed test cycle #{self.test_counter}")
    
        # Increment phase counter
        self.test_phase += 1

    def cmd_vel_callback(self, msg):
        """Convert Twist message to motor commands"""
        # Basic differential drive conversion
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Simple conversion for differential drive
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
            # Parse commands like "status,left_speed,right_speed"
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

    def send_motor_command(self, status, left_speed, right_speed):
        """Send motor command to ESP32"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warning("Serial connection not available, attempting to reconnect...")
            self.connect_serial()
            return
            
        try:
            # Create command string in format "status,left_speed,right_speed"
            command = f"{status},{left_speed},{right_speed}\n"
            self.serial_conn.write(command.encode('utf-8'))
            self.get_logger().debug(f"Sent motor command: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Error sending motor command: {e}")
            self.connect_serial()
    
    def read_imu_data(self):
        """Read and process IMU data from serial"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warning("Serial connection not available, attempting to reconnect...")
            self.connect_serial()
            return
            
        if self.serial_conn.in_waiting >= 22:  # Minimum size of the expected binary message
            try:
                # Read binary data from serial port
                data = self.serial_conn.read(22)  # Read 22 bytes (size of the full message)
                
                # Parse binary data
                mcu_state = data[0]
                host_state = data[1]
                
                # Wheel distances (32-bit values)
                wheel1_distance = int.from_bytes(data[2:6], byteorder='big', signed=False)
                wheel2_distance = int.from_bytes(data[6:10], byteorder='big', signed=False)
                
                # Extract IMU data (16-bit values, convert to proper units)
                # Acceleration values (convert to m/s^2)
                acc_x = self.bytes_to_int16(data[10:12]) / 16384.0 * 9.81
                acc_y = self.bytes_to_int16(data[12:14]) / 16384.0 * 9.81
                acc_z = self.bytes_to_int16(data[14:16]) / 16384.0 * 9.81
                
                # Gyroscope values (convert to rad/s)
                gyr_x = self.bytes_to_int16(data[16:18]) / 131.0 * (3.14159265359/180.0)
                gyr_y = self.bytes_to_int16(data[18:20]) / 131.0 * (3.14159265359/180.0)
                gyr_z = self.bytes_to_int16(data[20:22]) / 131.0 * (3.14159265359/180.0)
                
                # Create IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.frame_id
                
                # Orientation (we don't receive quaternion data from MCU)
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = 0.0
                imu_msg.orientation.z = 0.0
                imu_msg.orientation.w = 1.0
                
                # Angular velocity (rad/s)
                imu_msg.angular_velocity.x = gyr_x
                imu_msg.angular_velocity.y = gyr_y
                imu_msg.angular_velocity.z = gyr_z
                
                # Linear acceleration (m/s^2)
                imu_msg.linear_acceleration.x = acc_x
                imu_msg.linear_acceleration.y = acc_y
                imu_msg.linear_acceleration.z = acc_z
                
                # Covariance (adjust based on sensor specs)
                imu_msg.orientation_covariance = self.create_covariance_array(0.01)
                imu_msg.angular_velocity_covariance = self.create_covariance_array(0.001)
                imu_msg.linear_acceleration_covariance = self.create_covariance_array(0.005)
                
                # Publish IMU message
                self.imu_pub.publish(imu_msg)
                
                # Debug info
                self.get_logger().debug(f"Published IMU data: a=[{acc_x:.2f}, {acc_y:.2f}, {acc_z:.2f}], "
                                      f"w=[{gyr_x:.2f}, {gyr_y:.2f}, {gyr_z:.2f}]")
                
            except Exception as e:
                self.get_logger().error(f"Error processing IMU data: {e}")
                # Try to flush buffer on error
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.reset_input_buffer()
    
    def bytes_to_int16(self, data_bytes):
        """Convert 2 bytes to signed 16-bit integer"""
        value = (data_bytes[0] << 8) | data_bytes[1]
        # Convert to signed (2's complement)
        if value > 32767:
            value -= 65536
        return value
    
    def create_covariance_array(self, diagonal_value):
        """Create covariance array with specified value on diagonal"""
        return [float(diagonal_value if i % 4 == 0 else 0) for i in range(9)]
        
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
    node = ESP32CombinedNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()