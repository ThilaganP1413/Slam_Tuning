"""
offline_bag_slam_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Bag path: point directly at the .db3 file to avoid metadata.yaml ─────
    # parsing issues caused by type_description_hash formatting in Humble.
    _this_file = os.path.abspath(__file__)
    _ws_from_install = os.path.realpath(os.path.join(_this_file, *(['..'] * 6)))
    _ws_from_source  = os.path.realpath(os.path.join(_this_file, *(['..'] * 3)))

    _db3_from_install = os.path.join(_ws_from_install, 'src', 'bag', 'converted_bag', 'converted_bag.db3')
    _db3_from_source  = os.path.join(_ws_from_source,  'bag', 'converted_bag', 'converted_bag.db3')

    if os.path.exists(_db3_from_install):
        _default_bag = _db3_from_install
    elif os.path.exists(_db3_from_source):
        _default_bag = _db3_from_source
    else:
        _default_bag = _db3_from_install  # show expected path in error

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=_default_bag,
        description='Absolute path to the .db3 bag file',
    )

    slam_params_arg = DeclareLaunchArgument(
        'slam_params',
        default_value=os.path.join(
            get_package_share_directory('slam_toolbox'),
            'config', 'mapper_params_offline.yaml',
        ),
        description='Path to slam_toolbox mapper params yaml',
    )

    bag_path    = LaunchConfiguration('bag_path')
    slam_params = LaunchConfiguration('slam_params')

    # ── TF + odom node: start immediately ────────────────────────────────────
    tf_odom_node = Node(
        package='bag_mapping',
        executable='tf_and_odom',
        name='tf_and_odom_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ── SLAM node: start immediately, before bag data arrives ────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
        remappings=[('/scan', '/scan')],
    )

    # ── Bag play: delayed so all nodes are fully initialised first ────────────
    # 3 s gives slam_toolbox time to load its params and subscribe before
    # the first /scan and /odom messages are published.
    bag_play = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_path, '--clock', '--rate', '1.0'],
                output='screen',
            )
        ],
    )

    return LaunchDescription([bag_path_arg, slam_params_arg, tf_odom_node, slam_node, bag_play])