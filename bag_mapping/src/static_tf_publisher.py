#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticTfPublisher(Node):

    def __init__(self):
        super().__init__('static_tf_publisher')

        self._sent = False
        self._broadcaster = StaticTransformBroadcaster(self)

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos)
        self.get_logger().info('Waiting for first /scan to stamp static TF…')

    def _scan_cb(self, msg: LaserScan):
        if self._sent:
            return

        tf = TransformStamped()
        tf.header.stamp    = msg.header.stamp   # use bag timestamp
        tf.header.frame_id = 'base_link'
        tf.child_frame_id  = 'rslidar_laser'
        tf.transform.rotation.w = 1.0           # identity rotation, zero offset

        self._broadcaster.sendTransform(tf)
        self._sent = True
        self.get_logger().info(
            f'Published static TF base_link → rslidar_laser '
            f'(stamp {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()