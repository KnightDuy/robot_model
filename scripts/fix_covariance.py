#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class FixCovariance(Node):
    def __init__(self):
        super().__init__('fix_covariance')
        
        self.odom_pub = self.create_publisher(Odometry, '/odom_fixed', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.imu_pub = self.create_publisher(Imu, '/imu_fixed', 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.get_logger().info('Fixing sensor covariances for EKF')
    
    def odom_callback(self, msg):
        msg.pose.covariance[0] = 0.001
        msg.pose.covariance[7] = 0.001
        msg.pose.covariance[14] = 1000000.0
        msg.pose.covariance[21] = 1000000.0
        msg.pose.covariance[28] = 1000000.0
        msg.pose.covariance[35] = 0.03
        
        msg.twist.covariance[0] = 0.001
        msg.twist.covariance[7] = 0.001
        msg.twist.covariance[14] = 1000000.0
        msg.twist.covariance[21] = 1000000.0
        msg.twist.covariance[28] = 1000000.0
        msg.twist.covariance[35] = 0.03
        
        self.odom_pub.publish(msg)
    
    def imu_callback(self, msg):
        msg.orientation_covariance[0] = 0.01
        msg.orientation_covariance[4] = 0.01
        msg.orientation_covariance[8] = 0.01
        
        msg.angular_velocity_covariance[0] = 0.001
        msg.angular_velocity_covariance[4] = 0.001
        msg.angular_velocity_covariance[8] = 0.001
        
        msg.linear_acceleration_covariance[0] = 0.01
        msg.linear_acceleration_covariance[4] = 0.01
        msg.linear_acceleration_covariance[8] = 0.01
        
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
