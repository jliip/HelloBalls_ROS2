#!/usr/bin/env python3

import rclpy # type: ignore
from rclpy.node import Node # type: ignore
import serial # type: ignore
import json 
from sensor_msgs.msg import Imu # type: ignore
from geometry_msgs.msg import Vector3 # type: ignore
import math
import time

class ESP32ImuNode(Node):
    def __init__(self):
        super().__init__('esp32_imu_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyS1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 100.0)  # Hz
        self.declare_parameter('connect_retry_period', 5.0)  # seconds
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.connect_retry_period = self.get_parameter('connect_retry_period').value
        
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
            
        if self.serial_conn.in_waiting:
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
                imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
                
                # Angular velocity covariance
                imu_msg.angular_velocity_covariance = [0.001, 0, 0, 0, 0.001, 0, 0, 0, 0.001]
                
                # Linear acceleration covariance
                imu_msg.linear_acceleration_covariance = [0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005]
                
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

def main():
    print('Hi from esp32_imu_node.')


if __name__ == '__main__':
    main()
