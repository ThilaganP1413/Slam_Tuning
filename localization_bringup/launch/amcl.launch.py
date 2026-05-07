
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition,UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directories
    pkg_share = FindPackageShare('localization_bringup').find('localization_bringup')

    # Files paths
    rviz_config = os.path.join(pkg_share, 'rviz/slam.rviz')

    # Launch configuration 
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    sync =LaunchConfiguration('sync')
    localization = LaunchConfiguration('localization')


    # Launch Arguments 
    declare_arguments = [

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),
    ]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=['/home/education-robot/robot/tools/slamtoolbox/src/localization_bringup/config/amcl.yaml']
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=['/home/education-robot/robot/tools/slamtoolbox/src/localization_bringup/config/amcl.yaml']
    )


    return LaunchDescription(
        declare_arguments + [
            map_server_node,
            amcl_node
        ]
    )
