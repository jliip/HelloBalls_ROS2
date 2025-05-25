#!/usr/bin/env python3

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
import serial # type: ignore
import json
import struct
from sensor_msgs.msg import Imu # type: ignore
from geometry_msgs.msg import Vector3 # type: ignore
import math
import time

class ESP32ImuNode(Node):
    def __init__(self):
        super().__init__('esp32_imu_node')
        
        # Helper function to create properly formatted covariance matrices
        self.create_covariance_array = lambda diagonal_value: [float(diagonal_value if i % 4 == 0 else 0) for i in range(9)]
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyS1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('connect_retry_period', 5.0)  # seconds
        self.declare_parameter('use_binary_format', True)  # Set to True for binary format, False for JSON
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.connect_retry_period = self.get_parameter('connect_retry_period').value
        self.use_binary_format = self.get_parameter('use_binary_format').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.connect_serial()
        
        # Create publisher
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        
        # Create timer for reading from serial
        self.timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(self.timer_period, self.read_imu_data)
        
        self.get_logger().info(f"ESP32 IMU Node initialized with {self.serial_port} at {self.baud_rate} baud")
        self.get_logger().info(f"Publishing on /imu/data at {self.publish_rate} Hz")

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
                self.get_logger().info(f"Retrying in {self.connect_retry_period} seconds...")
                time.sleep(self.connect_retry_period)
        return False

    def read_imu_data(self):
        """Read and process IMU data from serial"""
        if not self.serial_conn or not self.serial_conn.is_open:
            self.get_logger().warning("Serial connection not available, attempting to reconnect...")
            self.connect_serial()
            return
            
        if self.use_binary_format and self.serial_conn.in_waiting >= 22:  # Minimum size of the expected binary message
            try:
                # Read binary data from serial port
                data = self.serial_conn.read(22)  # Read 22 bytes (size of the full message)
                
                # Parse binary data
                mcu_state = data[0]
                host_state = data[1]
                
                # Wheel distances (32-bit values)
                wheel1_distance = int.from_bytes(data[2:6], byteorder='big', signed=False)
                wheel2_distance = int.from_bytes(data[6:10], byteorder='big', signed=False)
                
                # Helper function to convert 2-byte data to signed 16-bit int
                def bytes_to_int16(data_bytes):
                    value = (data_bytes[0] << 8) | data_bytes[1]
                    # Convert to signed (2's complement)
                    if value > 32767:
                        value -= 65536
                    return value
                
                # Extract IMU data (16-bit values, convert to proper units)
                # Acceleration values (convert to m/s^2)
                acc_x = bytes_to_int16(data[10:12]) / 16384.0 * 9.81  # Assuming sensitivity of 16384 LSB/g
                acc_y = bytes_to_int16(data[12:14]) / 16384.0 * 9.81
                acc_z = bytes_to_int16(data[14:16]) / 16384.0 * 9.81
                
                # Gyroscope values (convert to rad/s)
                gyr_x = bytes_to_int16(data[16:18]) / 131.0 * (3.14159265359/180.0)  # Assuming sensitivity of 131 LSB/deg/s
                gyr_y = bytes_to_int16(data[18:20]) / 131.0 * (3.14159265359/180.0)
                gyr_z = bytes_to_int16(data[20:22]) / 131.0 * (3.14159265359/180.0)
                
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
                # Using moderate covariance values for orientation
                imu_msg.orientation_covariance = self.create_covariance_array(0.01)
                
                # Angular velocity covariance
                imu_msg.angular_velocity_covariance = self.create_covariance_array(0.001)
                
                # Linear acceleration covariance
                imu_msg.linear_acceleration_covariance = self.create_covariance_array(0.005)
                
                # Publish IMU message
                self.imu_pub.publish(imu_msg)
                
                # Debug info
                self.get_logger().debug(f"Published IMU data: a=[{imu_msg.linear_acceleration.x:.2f}, "
                                       f"{imu_msg.linear_acceleration.y:.2f}, {imu_msg.linear_acceleration.z:.2f}], "
                                       f"w=[{imu_msg.angular_velocity.x:.2f}, {imu_msg.angular_velocity.y:.2f}, "
                                       f"{imu_msg.angular_velocity.z:.2f}]")
                
                # Additional debug info for wheel encoders and state values
                self.get_logger().debug(f"Wheel distances: [{wheel1_distance}, {wheel2_distance}], "
                                      f"States: MCU={mcu_state}, Host={host_state}")
                
            except IndexError as e:
                self.get_logger().warn(f"Failed to parse IMU data (incomplete data): {e}")
                # Try to flush the serial buffer to recover from this error
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.reset_input_buffer()
            except struct.error as e:
                self.get_logger().warn(f"Binary data parsing error: {e}")
                if self.serial_conn and self.serial_conn.is_open:
                    self.serial_conn.reset_input_buffer()
            except Exception as e:
                self.get_logger().error(f"Error processing IMU data: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
        
        # JSON format handling (original code)
        elif not self.use_binary_format and self.serial_conn.in_waiting:
            try:
                # Read line from serial port
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                # Parse JSON data
                data = json.loads(line)
                
                # Create IMU message
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = self.frame_id
                
                # Orientation (set to zeroes if not provided by ESP32)
                imu_msg.orientation.x = data.get('quat_x', 0.0)
                imu_msg.orientation.y = data.get('quat_y', 0.0)
                imu_msg.orientation.z = data.get('quat_z', 0.0)
                imu_msg.orientation.w = data.get('quat_w', 1.0)
                
                # Angular velocity (rad/s)
                imu_msg.angular_velocity.x = data.get('gyro_x', 0.0)
                imu_msg.angular_velocity.y = data.get('gyro_y', 0.0)
                imu_msg.angular_velocity.z = data.get('gyro_z', 0.0)
                
                # Linear acceleration (m/s^2)
                imu_msg.linear_acceleration.x = data.get('accel_x', 0.0)
                imu_msg.linear_acceleration.y = data.get('accel_y', 0.0)
                imu_msg.linear_acceleration.z = data.get('accel_z', 0.0)
                
                # Covariance (adjust based on sensor specs)
                # Using moderate covariance values for orientation
                imu_msg.orientation_covariance = self.create_covariance_array(0.01)
                
                # Angular velocity covariance
                imu_msg.angular_velocity_covariance = self.create_covariance_array(0.001)
                
                # Linear acceleration covariance
                imu_msg.linear_acceleration_covariance = self.create_covariance_array(0.005)
                
                # Publish IMU message
                self.imu_pub.publish(imu_msg)
                
                # Debug info
                self.get_logger().debug(f"Published IMU data: a=[{imu_msg.linear_acceleration.x:.2f}, "
                                       f"{imu_msg.linear_acceleration.y:.2f}, {imu_msg.linear_acceleration.z:.2f}], "
                                       f"w=[{imu_msg.angular_velocity.x:.2f}, {imu_msg.angular_velocity.y:.2f}, "
                                       f"{imu_msg.angular_velocity.z:.2f}]")
                
            except json.JSONDecodeError as e:
                self.get_logger().warn(f"Failed to parse IMU data: {e}")
            except UnicodeDecodeError as e:
                self.get_logger().warn(f"Unicode decode error: {e}")
            except Exception as e:
                self.get_logger().error(f"Error processing IMU data: {e}")

    def __del__(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info("Serial connection closed")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32ImuNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
