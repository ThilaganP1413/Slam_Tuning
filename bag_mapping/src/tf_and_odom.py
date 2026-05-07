#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TfAndOdomPublisher(Node):

    def __init__(self):
        super().__init__('tf_and_odom_publisher')

        self._static_broadcaster = StaticTransformBroadcaster(self)
        self._tf_broadcaster     = TransformBroadcaster(self)
        self._static_tf_sent     = False

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Subscribe to both /odom and /scan so whichever arrives first
        # in the bag can seed the static TF timestamp.
        self.create_subscription(Odometry,  '/odom', self._odom_callback, qos)
        self.create_subscription(LaserScan, '/scan', self._scan_callback, qos)
        self.get_logger().info('Subscribed to /odom and /scan')

    # ── Static TF (sent once, stamped from the bag) ───────────────────────────
    def _maybe_send_static_tf(self, stamp):
        if self._static_tf_sent:
            return
        static_tf = TransformStamped()
        static_tf.header.stamp    = stamp   # first timestamp seen from the bag
        static_tf.header.frame_id = 'base_link'
        static_tf.child_frame_id  = 'rslidar_laser'
        static_tf.transform.translation.x = 0.0
        static_tf.transform.translation.y = 0.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.w    = 1.0
        self._static_broadcaster.sendTransform(static_tf)
        self._static_tf_sent = True
        self.get_logger().info(
            f'Published static TF: base_link -> rslidar_laser '
            f'(stamp {stamp.sec}.{stamp.nanosec})'
        )

    # ── /scan callback: only used to seed the static TF early ────────────────
    def _scan_callback(self, msg: LaserScan):
        self._maybe_send_static_tf(msg.header.stamp)

    # ── /odom callback: seeds static TF + rebroadcasts odom → base_link ──────
    def _odom_callback(self, msg: Odometry):
        self._maybe_send_static_tf(msg.header.stamp)

        tf = TransformStamped()
        tf.header.stamp    = msg.header.stamp
        tf.header.frame_id = msg.header.frame_id
        tf.child_frame_id  = msg.child_frame_id
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation      = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = TfAndOdomPublisher()
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