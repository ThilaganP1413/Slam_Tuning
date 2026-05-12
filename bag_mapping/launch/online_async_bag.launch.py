import os
from launch.actions import ExecuteProcess, TimerAction
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("bag_mapping"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')
    
    bag_path = os.path.expanduser('~/ROS2/slam_ws/bag/patrol_lvl3_odom')
    
    bag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play',
            bag_path,
            '--clock'],
        output='screen'
    )

    # static_lidar_tf = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_lidar_tf',
    #     arguments=['0.55', '0', '0', '0', '0', '0', 'base_link', 'lidar'],
    #     parameters=[{'use_sim_time': use_sim_time}]
    # )
    
    odom_pub = Node(
        package='bag_mapping',
        executable='odom_pub',
        name='odom_pub',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        arguments=['--ros-args', '--log-level', 'slam_toolbox:=debug']
        )

    ld = LaunchDescription()

    ld.add_action(bag_play)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(TimerAction(period=2.0, actions=[
        start_async_slam_toolbox_node,
        # static_lidar_tf,
        odom_pub,
    ]))
    return ld