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

    _bag_from_install = os.path.join(
        _ws_from_install, 'src', 'bag', 'patrol_lvl3_new', 'bag_20260421_165900')
    _bag_from_source  = os.path.join(
        _ws_from_source, 'bag', 'patrol_lvl3_new', 'bag_20260421_165900')

    _default_bag = (
        _bag_from_install if os.path.exists(_bag_from_install) else
        _bag_from_source  if os.path.exists(_bag_from_source)  else
        _bag_from_install   # fallback — gives a clear error if missing
    )

    _pkg_bag_mapping = get_package_share_directory('bag_mapping')

    # ── Launch arguments ──────────────────────────────────────────────────────
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=_default_bag,
        description='Absolute path to the bag FOLDER (not .db3 file)',
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params',
        default_value=os.path.join(
            _pkg_bag_mapping, 'config', 'slamware_mapper_params_offline.yaml'),
        description='Path to slam_toolbox mapper params yaml',
    )

    bag_path    = LaunchConfiguration('bag_path')
    slam_params = LaunchConfiguration('slam_params')

    # ─────────────────────────────────────────────────────────────────────────
    # 1. TF + odom publisher (slamware_tf_pub)
    #    • Subscribes  : /our_pose, /laser_scan
    #    • Publishes TF: slmtb_odom -> slmtb_base_link  (dynamic, from /our_pose)
    #    • Publishes TF: slmtb_base_link -> lidar_frame  (static, from bag TF)
    #    • Republishes : /laser_scan → /scan  (frame_id remapped to lidar_frame)
    # ─────────────────────────────────────────────────────────────────────────
    tf_odom_node = Node(
        package='bag_mapping',
        executable='slamware_tf_pub',
        name='slamware_tf_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 2. slam_toolbox – async offline mapping
    #    Expects:
    #      /scan            — laser scans  (published by slamware_tf_pub)
    #      /tf              — odom → base_link chain
    #      /clock           — sim time from bag
    # ─────────────────────────────────────────────────────────────────────────
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
        remappings=[('/map', '/slmtb_map')],
    )

    # ─────────────────────────────────────────────────────────────────────────
    # 3. Bag playback
    #    Delayed 8 s so all nodes are fully subscribed to /clock before
    #    sim-time starts advancing.
    #    Pass the FOLDER path — ros2 bag play locates the .db3 automatically.
    # ─────────────────────────────────────────────────────────────────────────
    bag_play = TimerAction(
        period=8.0,            # ← increased from 5 s to 8 s
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
        tf_odom_node,
        slam_node,
        bag_play,
    ])