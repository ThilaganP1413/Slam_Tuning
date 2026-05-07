
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Packages share directories
    pkg_share = FindPackageShare('localization_bringup').find('localization_bringup')

    # File paths
    ekf_config = os.path.join(pkg_share, 'config/ekf.yaml')

    # Launch configuration variables with default values
    use_sim_time = LaunchConfiguration('use_sim_time')


    # Launch Arguments (used to modify at launch time)
    declare_arguments = [
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Simulation(Gazebo) Clock'
        ),

    ]
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription(
        declare_arguments + [
            ekf_node
        ]
    )
