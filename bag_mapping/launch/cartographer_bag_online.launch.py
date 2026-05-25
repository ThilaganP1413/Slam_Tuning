from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Update these paths to match your actual directory structure
    urdf_path = os.path.expanduser('~/ROS2/slam_ws/src/bag_mapping/urdf/ipr_robot.urdf')
    config_path = os.path.expanduser('~/ROS2/slam_ws/src/localization_bringup/config/cartographer.lua')

    return LaunchDescription([
        # Node(
        # package='bag_mapping',
        # executable='cartographer_odom_pub',
        # name='cartographer_odom_pub',
        # output='screen',
        # parameters=[{'use_sim_time': True}],
        # ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read(), 'use_sim_time': True}]
        ),
        
        # Start Cartographer
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-configuration_directory', os.path.dirname(config_path),
                       '-configuration_basename', 'cartographer.lua'],
            # This is the line that fixes your topic issue:
            # remappings=[('/scan', '/laser_scan')] 
        ),
        
        # Occupancy Grid Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': True, 'resolution': 0.05}]
        )
    ])