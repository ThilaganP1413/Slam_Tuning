"""
offline_bag_slam_ekf_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Workspace / bag path resolution ──────────────────────────────────────
    _this_file = os.path.abspath(__file__)
    _ws_from_install = os.path.realpath(os.path.join(_this_file, *(['..'] * 6)))
    _ws_from_source  = os.path.realpath(os.path.join(_this_file, *(['..'] * 3)))

    _db3_from_install = os.path.join(
        _ws_from_install, 'src', 'bag', 'converted_bag', 'converted_bag.db3')
    _db3_from_source  = os.path.join(
        _ws_from_source, 'bag', 'converted_bag', 'converted_bag.db3')

    _default_bag = (
        _db3_from_install if os.path.exists(_db3_from_install) else
        _db3_from_source  if os.path.exists(_db3_from_source)  else
        _db3_from_install
    )

    _pkg_bag_mapping = get_package_share_directory('bag_mapping')

    _ekf_from_bag_mapping = os.path.join(
        _pkg_bag_mapping, 'config', 'ekf_params.yaml')
    _ekf_from_rl = os.path.join(
        get_package_share_directory('robot_localization'),
        'params', 'ekf.yaml')

    _default_ekf = (
        _ekf_from_bag_mapping if os.path.exists(_ekf_from_bag_mapping)
        else _ekf_from_rl
    )

    # ── Launch arguments ─────────────────────────────────────────────────────
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=_default_bag,
        description='Absolute path to the .db3 bag file',
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params',
        default_value=os.path.join(
            _pkg_bag_mapping, 'config', 'mapper_params_online_sync.yaml'),
        description='Path to slam_toolbox mapper_params yaml',
    )
    ekf_params_arg = DeclareLaunchArgument(
        'ekf_params',
        default_value=_default_ekf,
        description='Path to robot_localization EKF params yaml',
    )

    bag_path    = LaunchConfiguration('bag_path')
    slam_params = LaunchConfiguration('slam_params')
    ekf_params  = LaunchConfiguration('ekf_params')

    # ─────────────────────────────────────────────────────────────────────────
    # 1. Static TF: base_link → rslidar_laser
    #    Uses a Python node instead of static_transform_publisher so the
    #    transform is stamped with the first /scan timestamp from the bag.
    #    static_transform_publisher always uses wall-clock time, which is
    #    ~5 years ahead of the 2022 bag timestamps, causing TF cache misses.
    # ─────────────────────────────────────────────────────────────────────────
    static_tf_node = Node(
        package='bag_mapping',
        executable='static_tf_publisher',
        name='static_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 2. robot_localization EKF: odom → base_link TF
    #    Fuses /odom (x, y, yaw-rate) + /imu (orientation, gyro, accel)
    # ─────────────────────────────────────────────────────────────────────────
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params, {'use_sim_time': True}],
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 3. slam_toolbox – offline synchronous mapping
    # ─────────────────────────────────────────────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
        remappings=[('/scan', '/scan')],
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 4. Bag playback – delayed 5 s so all nodes are fully initialised
    # ─────────────────────────────────────────────────────────────────────────
    bag_play = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'bag', 'play', bag_path,
                    '--clock',
                    '--rate', '1.0',
                ],
                output='screen',
            )
        ],
    )

    return LaunchDescription([
        bag_path_arg,
        slam_params_arg,
        ekf_params_arg,
        static_tf_node,
        ekf_node,
        slam_node,
        bag_play,
    ])