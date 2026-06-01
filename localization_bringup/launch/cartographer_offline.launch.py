import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory("localization_bringup")

    # Path to configuration file
    configuration_basename = LaunchConfiguration(
        "configuration_basename", default="cartographer.lua"
    )
    
    config_dir = LaunchConfiguration(
        "config_dir", default=os.path.join(pkg_share, "config")
    )

    urdf_file = LaunchConfiguration(
        "urdf_file", default=os.path.join(pkg_share, "urdf", "robot.urdf")
    )

    # Bag file path - NOTE: Must point to the .db3 file directly
    bag_filename = LaunchConfiguration(
        "bag_filename", 
        default=""
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "configuration_basename",
                default_value="cartographer.lua",
                description="Name of lua configuration file",
            ),
            DeclareLaunchArgument(
                "config_dir",
                default_value=os.path.join(pkg_share, "config"),
                description="Full path to config directory",
            ),
            DeclareLaunchArgument(
                "urdf_file",
                default_value=os.path.join(pkg_share, "urdf", "robot.urdf"),
                description="Full path to URDF file",
            ),
            DeclareLaunchArgument(
                "bag_filename",
                default_value="/home/thilaks/ROS2/slam_ws/bag/IPR2/IPR2_raw_odom_patrol_edited.db3",
                description="Full path to bag file (.db3)",
            ),
            # Cartographer offline node - processes bag directly
            Node(
                package="cartographer_ros",
                executable="cartographer_offline_node",
                name="cartographer_offline_node",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "-configuration_directory", config_dir,
                    "-configuration_basenames",
                    configuration_basename,
                    "-bag_filenames",
                    bag_filename,
                    "-urdf_filenames",
                    urdf_file,
                    "--ros-args",
                ],
            ),
        ]
    )