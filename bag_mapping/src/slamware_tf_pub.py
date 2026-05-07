#!/usr/bin/env python3
"""
slamware_tf_pub.py

Fixes applied
─────────────
1. Static TF (slmtb_base_link → lidar_frame) is now looked up using the
   bag's own sim-time stamp (taken from the first incoming /laser_scan
   message) instead of the node wall-clock.  This eliminates the
   "extrapolation into the future" error that prevented the TF from ever
   being published.

2. The timer-based _try_publish_lidar_tf() is kept only as a one-shot
   safety net; the primary path is driven from scan_callback() so the
   lookup timestamp always matches an existing entry in the TF buffer.

3. All other timestamps already used msg.header.stamp (bag time) —
   those are left unchanged.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped, PoseStamped
from sensor_msgs.msg import LaserScan

import tf2_ros
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    Buffer,
    TransformListener,
)


# ── Quaternion helpers ────────────────────────────────────────────────────────

def quat_multiply(q1, q2):
    x1, y1, z1, w1 = q1['x'], q1['y'], q1['z'], q1['w']
    x2, y2, z2, w2 = q2['x'], q2['y'], q2['z'], q2['w']
    return {
        'x':  w1*x2 + x1*w2 + y1*z2 - z1*y2,
        'y':  w1*y2 - x1*z2 + y1*w2 + z1*x2,
        'z':  w1*z2 + x1*y2 - y1*x2 + z1*w2,
        'w':  w1*w2 - x1*x2 - y1*y2 - z1*z2,
    }


def quat_conjugate(q):
    return {'x': -q['x'], 'y': -q['y'], 'z': -q['z'], 'w': q['w']}


def rotate_vector_by_quat(v, q):
    v_quat  = {'x': v[0], 'y': v[1], 'z': v[2], 'w': 0.0}
    q_conj  = quat_conjugate(q)
    tmp     = quat_multiply(q, v_quat)
    result  = quat_multiply(tmp, q_conj)
    return result['x'], result['y'], result['z']


# ── Node ──────────────────────────────────────────────────────────────────────

class SlamwareTFPub(Node):

    def __init__(self):
        super().__init__('slamware_tf_pub')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('pub_parent_frame',   'slmtb_odom')
        self.declare_parameter('pub_child_frame',    'slmtb_base_link')
        self.declare_parameter('lidar_frame',        'lidar_frame')
        self.declare_parameter('source_base_frame',  'base_link')
        self.declare_parameter('source_lidar_frame', 'lidar')

        self.pub_parent         = self.get_parameter('pub_parent_frame').value
        self.pub_child          = self.get_parameter('pub_child_frame').value
        self.lidar_frame        = self.get_parameter('lidar_frame').value
        self.source_base_frame  = self.get_parameter('source_base_frame').value
        self.source_lidar_frame = self.get_parameter('source_lidar_frame').value

        # ── State ─────────────────────────────────────────────────────────────
        self._origin              = None
        self._lidar_tf_published  = False

        # ── TF infrastructure ─────────────────────────────────────────────────
        self.tf_buffer             = Buffer()
        self.tf_listener           = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster        = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # Safety-net timer: fires every second in case scans arrive before the
        # TF buffer is populated.  Cancelled as soon as the TF is published.
        self._lidar_tf_timer = self.create_timer(1.0, self._timer_try_publish_lidar_tf)

        # ── Subscriptions / publishers ─────────────────────────────────────────
        self.pose_sub = self.create_subscription(
            PoseStamped, '/our_pose', self.pose_callback, 10)

        self.scan_sub = self.create_subscription(
            LaserScan, '/laser_scan', self.scan_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info(
            f'SlamwareTFPub started.\n'
            f'  Subscribing to  : /our_pose\n'
            f'  Publishing TF   : {self.pub_parent} -> {self.pub_child}\n'
            f'  Will look up TF : {self.source_base_frame} -> {self.source_lidar_frame}\n'
            f'  Re-publish as   : {self.pub_child} -> {self.lidar_frame}\n'
            f'  Waiting for first pose to set origin...'
        )

    # ── Static lidar TF helpers ───────────────────────────────────────────────

    def _try_publish_lidar_tf_at(self, stamp: Time) -> None:
        """
        Look up base_link → lidar at *stamp* (a bag sim-time) and republish
        the result as the static TF  pub_child → lidar_frame.

        Using the message's own timestamp means the entry is guaranteed to
        exist in the TF buffer, avoiding the "extrapolation into the future"
        error that occurs when the node wall-clock is used instead.
        """
        if self._lidar_tf_published:
            return

        try:
            src: TransformStamped = self.tf_buffer.lookup_transform(
                self.source_base_frame,
                self.source_lidar_frame,
                stamp,                        # ← bag sim-time, not wall clock
                timeout=Duration(seconds=0.5),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warn(
                f'Waiting for TF {self.source_base_frame} -> '
                f'{self.source_lidar_frame}: {e}',
                throttle_duration_sec=5.0,
            )
            return

        t = TransformStamped()
        t.header.stamp    = stamp.to_msg()   # bag sim-time stamp
        t.header.frame_id = self.pub_child
        t.child_frame_id  = self.lidar_frame
        t.transform       = src.transform    # copy the whole transform at once

        self.static_tf_broadcaster.sendTransform(t)
        self._lidar_tf_published = True

        self.get_logger().info(
            f'Published static TF: {self.pub_child} -> {self.lidar_frame}  '
            f'translation=({t.transform.translation.x:.4f}, '
            f'{t.transform.translation.y:.4f}, '
            f'{t.transform.translation.z:.4f})'
        )

    def _timer_try_publish_lidar_tf(self) -> None:
        """
        Safety-net: fired by the 1 Hz timer.  Tries to look up the TF at the
        current sim-time.  Cancels itself once the TF has been published.
        """
        if self._lidar_tf_published:
            self._lidar_tf_timer.cancel()
            return

        # Use the current sim-time as reported by the node clock.
        # This will succeed once the bag has published at least one TF message.
        now = self.get_clock().now()
        self._try_publish_lidar_tf_at(now)

    # ── Pose callback ─────────────────────────────────────────────────────────

    def pose_callback(self, msg: PoseStamped) -> None:
        p = msg.pose

        # Always use the message's own bag timestamp.
        msg_stamp = Time.from_msg(msg.header.stamp)

        if self._origin is None:
            self._origin = {
                'tx': p.position.x,    'ty': p.position.y,    'tz': p.position.z,
                'qx': p.orientation.x, 'qy': p.orientation.y,
                'qz': p.orientation.z, 'qw': p.orientation.w,
            }
            self.get_logger().info(
                f'Origin set: pos=({self._origin["tx"]:.3f}, '
                f'{self._origin["ty"]:.3f}, {self._origin["tz"]:.3f})'
            )
            self._broadcast_relative(
                tx=0.0, ty=0.0, tz=0.0,
                qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                stamp=msg_stamp,
            )
            return

        q_origin = {
            'x': self._origin['qx'], 'y': self._origin['qy'],
            'z': self._origin['qz'], 'w': self._origin['qw'],
        }
        q_origin_inv = quat_conjugate(q_origin)
        q_current    = {
            'x': p.orientation.x, 'y': p.orientation.y,
            'z': p.orientation.z, 'w': p.orientation.w,
        }
        q_rel = quat_multiply(q_origin_inv, q_current)

        dx = p.position.x - self._origin['tx']
        dy = p.position.y - self._origin['ty']
        dz = p.position.z - self._origin['tz']
        rx, ry, rz = rotate_vector_by_quat((dx, dy, dz), q_origin_inv)

        self._broadcast_relative(
            tx=rx, ty=ry, tz=rz,
            qx=q_rel['x'], qy=q_rel['y'], qz=q_rel['z'], qw=q_rel['w'],
            stamp=msg_stamp,
        )
        self.get_logger().debug(
            f'Relative pose: pos=({rx:.3f}, {ry:.3f}, {rz:.3f})'
        )

    # ── Scan callback ─────────────────────────────────────────────────────────

    def scan_callback(self, msg: LaserScan) -> None:
        # PRIMARY path to publish the static lidar TF:
        # use the scan's own bag timestamp so the lookup is guaranteed to find
        # a matching entry in the TF buffer.
        if not self._lidar_tf_published:
            self._try_publish_lidar_tf_at(Time.from_msg(msg.header.stamp))

        # Keep the original bag timestamp — only remap the frame_id.
        msg.header.frame_id = self.lidar_frame
        self.scan_pub.publish(msg)

    # ── TF broadcast helper ───────────────────────────────────────────────────

    def _broadcast_relative(self, tx, ty, tz, qx, qy, qz, qw, stamp) -> None:
        t = TransformStamped()
        t.header.stamp    = stamp.to_msg()
        t.header.frame_id = self.pub_parent
        t.child_frame_id  = self.pub_child

        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = tz
        t.transform.rotation.x    = qx
        t.transform.rotation.y    = qy
        t.transform.rotation.z    = qz
        t.transform.rotation.w    = qw

        self.tf_broadcaster.sendTransform(t)


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = SlamwareTFPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()