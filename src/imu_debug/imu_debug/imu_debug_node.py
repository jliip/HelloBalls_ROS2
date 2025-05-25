#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuDebugNode(Node):
    def __init__(self):
        super().__init__('imu_debug_node')
        
        # Create subscriber to IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('IMU Debug Node initialized')
        self.get_logger().info('Subscribed to /imu/data topic')
        
    def imu_callback(self, msg):
        """Callback function for IMU messages"""
        self.get_logger().info('--------- IMU Data ---------')
        
        # Print linear acceleration
        self.get_logger().info(f'Linear Acceleration (m/s^2):'
                              f'\n  X: {msg.linear_acceleration.x:.4f}'
                              f'\n  Y: {msg.linear_acceleration.y:.4f}'
                              f'\n  Z: {msg.linear_acceleration.z:.4f}')
        
        # Print angular velocity
        self.get_logger().info(f'Angular Velocity (rad/s):'
                              f'\n  X: {msg.angular_velocity.x:.4f}'
                              f'\n  Y: {msg.angular_velocity.y:.4f}'
                              f'\n  Z: {msg.angular_velocity.z:.4f}')
        
        # Print orientation quaternion
        self.get_logger().info(f'Orientation (quaternion):'
                              f'\n  X: {msg.orientation.x:.4f}'
                              f'\n  Y: {msg.orientation.y:.4f}'
                              f'\n  Z: {msg.orientation.z:.4f}'
                              f'\n  W: {msg.orientation.w:.4f}')
        
        self.get_logger().info('---------------------------')

def main(args=None):
    rclpy.init(args=args)
    node = ImuDebugNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
