#!/usr/bin/env python3
import traceback

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # QoS
        qos = QoSProfile(depth=10)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos
        )

        self.get_logger().info('Odometry to TF converter started')

    def odom_callback(self, msg: Odometry):
        """Convert odometry message to TF transform"""
        try:
            # Create transform message
            t = TransformStamped()

            # Header - provide sensible defaults if absent
            t.header.stamp = msg.header.stamp
            t.header.frame_id = msg.header.frame_id if msg.header.frame_id else 'odom'
            # child_frame_id is a top-level field on Odometry
            t.child_frame_id = msg.child_frame_id if msg.child_frame_id else 'base_link'

            # Translation
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z

            # Rotation
            t.transform.rotation.x = msg.pose.pose.orientation.x
            t.transform.rotation.y = msg.pose.pose.orientation.y
            t.transform.rotation.z = msg.pose.pose.orientation.z
            t.transform.rotation.w = msg.pose.pose.orientation.w

            # Broadcast transform
            self.tf_broadcaster.sendTransform(t)

        except Exception:
            self.get_logger().error('Exception in odom_callback:\n' + traceback.format_exc())


def main(args=None):
    try:
        rclpy.init(args=args)
    except Exception:
        print('Failed to initialize rclpy')
        raise

    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        node.get_logger().error('Exception in spin:\n' + traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
