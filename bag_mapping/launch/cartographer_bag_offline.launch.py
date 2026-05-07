from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Update these paths to match your actual directory structure
    urdf_path = os.path.expanduser('~/ROS2/slam_ws/src/bag_mapping/urdf/ipr_robot.urdf')
    config_path = os.path.expanduser('~/ROS2/slam_ws/src/bag_mapping/config/cartographer_offline.lua')
    # Point directly to your bag folder
    bag_path = '/home/ubuntu-tp/ROS2/slam_ws/src/bag/patrol_lvl3_topic_updated'

    return LaunchDescription([
        # 1. State Publisher (Publishes your URDF transforms)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read(), 'use_sim_time': True}]
        ),
        
        # 2. Offline Mapping Node
        Node(
            package='cartographer_ros',
            executable='cartographer_offline_node',
            parameters=[{'use_sim_time': True}],
            arguments=[
                '-configuration_directory', os.path.dirname(config_path),
                '-configuration_basenames', 'cartographer_offline.lua',
                '-bag_filenames', bag_path,
            ],
            # remappings=[('/scan', '/laser_scan')] # Remapping your custom topic+
        ),
        
        # 3. Occupancy Grid Node (Needed to generate the map)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': True, 'resolution': 0.05}]
        )
    ])