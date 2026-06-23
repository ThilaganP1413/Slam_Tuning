#!/usr/bin/env python3
"""
localization_score_viewer.py
=============================

ROS 2 node that subscribes to:
  - /pose                  geometry_msgs/msg/PoseWithCovarianceStamped
  - /localization_quality  localization_quality_monitor/msg/LocalizationScore

and streams everything into Rerun (https://rerun.io):

  - Every pose received is plotted as a point (x, y - top-down).
    Its color is driven by the localization_score (int32, 0-100):
        0   -> red
        25  -> amber / yellow
        50  -> green
        75  -> light blue
        100 -> dark blue
    with a smooth gradient interpolated in between.
  - The view is a fixed top-down orthographic 2D view with a solid
    black background, and each pose point is drawn with radius 0.5.

SYNCHRONIZATION NOTE
---------------------
`LocalizationScore` has no header/stamp of its own, so there's nothing
to time-match it against a specific `/pose` message. This node keeps
the *latest* score it has seen in memory and uses that to color the
*next* incoming pose. That's normally fine since localization quality
changes slowly relative to pose rate. If you later add a header to
LocalizationScore and want exact synchronization instead, swap in a
message_filters.ApproximateTimeSynchronizer (a stub for this is left
commented out at the bottom of the file).

SETUP
-----
    pip install rerun-sdk

    # Make sure your workspace with localization_quality_monitor is
    # built and sourced, e.g.:
    colcon build --packages-select localization_quality_monitor
    source install/setup.bash

RUN
---
    python3 localization_score_viewer.py
    # or, if you drop this into a package's scripts/ and add it to
    # setup.py entry_points:
    ros2 run <your_pkg> localization_score_viewer.py
"""

from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from localization_quality_monitor.msg import LocalizationScore

import rerun as rr
import rerun.blueprint as rrb


# --------------------------------------------------------------------------
# Color gradient: 0 -> red ... 100 -> dark blue
# --------------------------------------------------------------------------
# (score, (R, G, B)) control points. Tweak freely.
_COLOR_STOPS: List[Tuple[float, Tuple[int, int, int]]] = [
    (0,   (211,  47,  47)),   # red            - bad
    (25,  (255, 179,   0)),   # amber / yellow
    (50,  ( 76, 175,  80)),   # green
    (75,  (  3, 169, 244)),   # light blue
    (100, ( 13,  27, 117)),   # dark blue      - good
]


def score_to_rgb(score: float) -> Tuple[int, int, int]:
    """Map a localization score in [0, 100] to an RGB color using the
    gradient defined in _COLOR_STOPS, linearly interpolating between
    whichever two control points the score falls between."""
    s = max(0.0, min(100.0, float(score)))

    for (s0, c0), (s1, c1) in zip(_COLOR_STOPS, _COLOR_STOPS[1:]):
        if s0 <= s <= s1:
            t = 0.0 if s1 == s0 else (s - s0) / (s1 - s0)
            r = round(c0[0] + t * (c1[0] - c0[0]))
            g = round(c0[1] + t * (c1[1] - c0[1]))
            b = round(c0[2] + t * (c1[2] - c0[2]))
            return (r, g, b)

    return _COLOR_STOPS[-1][1]  # fallback, shouldn't be reached


class LocalizationScoreViewer(Node):

    def __init__(self):
        super().__init__('localization_score_viewer')

        rr.init("localization_score_viewer", spawn=True)

        # Fixed top-down orthographic 2D view, solid black background.
        rr.send_blueprint(
            rrb.Blueprint(
                rrb.Spatial2DView(
                    origin="world",
                    name="Top-down",
                    background=rrb.Background(color=(0, 0, 0)),
                ),
                collapse_panels=True,
            )
        )

        # Full accumulated trail. We re-log the whole list each time a
        # pose arrives so Rerun shows a persistent, growing scatter
        # plot instead of a single point that overwrites itself.
        self._positions: List[Tuple[float, float]] = []
        self._colors: List[Tuple[int, int, int]] = []

        self._latest_score: Optional[float] = None

        # If your /pose publisher uses BEST_EFFORT (e.g. sensor-style
        # QoS), change the depth below to a QoSProfile with
        # ReliabilityPolicy.BEST_EFFORT instead of a plain int.
        self.create_subscription(
            PoseWithCovarianceStamped, '/pf_pose', self._pose_cb, 10
        )
        self.create_subscription(
            LocalizationScore, '/localization_quality', self._score_cb, 10
        )

        self.get_logger().info(
            'Listening on /pf_pose and /localization_quality, streaming to Rerun...'
        )

    # ------------------------------------------------------------------
    def _score_cb(self, msg: LocalizationScore) -> None:
        # Only the score itself is used (for coloring); metrics are not
        # plotted.
        self._latest_score = float(msg.localization_score)

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        rr.set_time("ros_time", timestamp=t if t > 0.0 else self._now())

        p = msg.pose.pose.position
        score = self._latest_score if self._latest_score is not None else 0.0
        color = score_to_rgb(score)

        self._positions.append((p.x, p.y))
        self._colors.append(color)

        rr.log(
            "world/pose_trail",
            rr.Points2D(positions=self._positions, colors=self._colors, radii=0.5),
        )

        # Extra: a point marking the most recent pose, drawn on top.
        rr.log(
            "world/current_pose",
            rr.Points2D(positions=[(p.x, p.y)], colors=[color], radii=0.5),
        )

    # ------------------------------------------------------------------
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main():
    rclpy.init()
    node = LocalizationScoreViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# --------------------------------------------------------------------------
# OPTIONAL: exact-time synchronization alternative
# --------------------------------------------------------------------------
# If LocalizationScore ever gets a `std_msgs/Header header` field, you can
# replace the two independent subscriptions above with this instead, so
# every plotted point uses the score from the *same instant* rather than
# the latest cached one:
#
#   import message_filters
#
#   pose_sub = message_filters.Subscriber(self, PoseWithCovarianceStamped, '/pose')
#   score_sub = message_filters.Subscriber(self, LocalizationScore, '/localization_quality')
#   ts = message_filters.ApproximateTimeSynchronizer(
#       [pose_sub, score_sub], queue_size=20, slop=0.05
#   )
#   ts.registerCallback(self._synced_cb)
#
#   def _synced_cb(self, pose_msg, score_msg):
#       ...  # same plotting logic, using score_msg.localization_score directly
