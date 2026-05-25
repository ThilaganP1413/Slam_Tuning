import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('localization_bringup')
    
    # Path to configuration file
    configuration_basename = LaunchConfiguration('configuration_basename', 
                                                  default='cartographer.lua')
    config_dir = LaunchConfiguration('config_dir', 
                                     default=os.path.join(pkg_share, 'config'))
    
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='cartographer.lua',
            description='Name of lua configuration file'),
        
        DeclareLaunchArgument(
            'config_dir',
            default_value=os.path.join(pkg_share, 'config'),
            description='Full path to config directory'),
        
        DeclareLaunchArgument(
            'resolution',
            default_value='0.05',
            description='Resolution of a grid cell in the published occupancy grid'),
        
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='OccupancyGrid publishing period'),

        # Cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('/odom', '/slamware_ros_sdk_server_node/odom'),
                ('/scan', '/slamware_ros_sdk_server_node/scan')
            ]
        ),

        # Occupancy grid node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'resolution': resolution},
                {'publish_period_sec': publish_period_sec}
            ]
        ),
    ])