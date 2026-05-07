#!/usr/bin/env python3

"""
cartographer_odom_pub.py

Subscribes to /our_pose (geometry_msgs/msg/PoseStamped) published by
Cartographer and republishes it as nav_msgs/msg/Odometry on /odom,
with parent frame 'odom' and child frame 'base_link'.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class CartographerOdomPub(Node):

    def __init__(self):
        super().__init__('cartographer_odom_pub')

        # ---------- QoS ----------
        # Match Cartographer's default Best Effort QoS so the subscription
        # can actually receive messages.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ---------- Subscriber ----------
        self.sub_pose = self.create_subscription(
            PoseStamped,
            '/our_pose',
            self.pose_callback,
            qos,
        )

        # ---------- Publisher ----------
        self.pub_odom = self.create_publisher(
            Odometry,
            '/odom',
            qos,
        )

        self.get_logger().info(
            'cartographer_odom_pub started.\n'
            '  Subscribing : /our_pose  (geometry_msgs/PoseStamped)\n'
            '  Publishing  : /odom      (nav_msgs/Odometry)\n'
            '  Frames      : odom -> base_link'
        )

    # ------------------------------------------------------------------
    def pose_callback(self, msg: PoseStamped) -> None:
        """Convert incoming PoseStamped to Odometry and publish."""

        odom = Odometry()

        # ---- Header ----
        # Preserve the original timestamp from Cartographer so downstream
        # nodes (e.g. robot_localization, Nav2) see a consistent time source.
        odom.header.stamp    = msg.header.stamp
        odom.header.frame_id = 'odom'          # parent / world frame

        # ---- Child frame ----
        odom.child_frame_id = 'base_link'

        # ---- Pose (position + orientation) ----
        odom.pose.pose.position.x  = msg.pose.position.x
        odom.pose.pose.position.y  = msg.pose.position.y
        odom.pose.pose.position.z  = msg.pose.position.z

        odom.pose.pose.orientation.x = msg.pose.orientation.x
        odom.pose.pose.orientation.y = msg.pose.orientation.y
        odom.pose.pose.orientation.z = msg.pose.orientation.z
        odom.pose.pose.orientation.w = msg.pose.orientation.w

        # Pose covariance (6x6 row-major).
        # Diagonal values set to a small but non-zero number so nav2 /
        # robot_localization accept the message without warnings.
        # Tune these to match your sensor's real uncertainty.
        odom.pose.covariance = [
            1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-3,
        ]

        # ---- Twist (velocity) ----
        # Cartographer's PoseStamped carries no velocity information.
        # Leave twist at zero; if you add a velocity estimator later,
        # populate odom.twist.twist.linear / angular here.
        odom.twist.covariance = [
            1e-3, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  1e-3, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e-3, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e-3, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e-3, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  1e-3,
        ]

        self.pub_odom.publish(odom)

        self.get_logger().debug(
            f'Published /odom  x={odom.pose.pose.position.x:.3f} '
            f'y={odom.pose.pose.position.y:.3f}  '
            f'qz={odom.pose.pose.orientation.z:.4f}  '
            f'qw={odom.pose.pose.orientation.w:.4f}'
        )


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = CartographerOdomPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()