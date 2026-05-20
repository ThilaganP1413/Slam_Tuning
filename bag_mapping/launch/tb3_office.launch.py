#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='7.0')
    y_pose = LaunchConfiguration('y_pose', default='5.0')
    z_pose = LaunchConfiguration('z_pose', default='0.1')
    frame_prefix = LaunchConfiguration('frame_prefix', default='')
    world = '/home/thilaks/ROS2/Gazebo_Worlds/gazebo_models_worlds_collection/worlds/office_env_large.world'

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    model_folder = 'turtlebot3_' + TURTLEBOT3_MODEL
    sdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models',
        model_folder,
        'model.sdf'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc,
            'frame_prefix': PythonExpression(["'", frame_prefix, "/'"])
        }],
    )

    spawn_turtlebot_cmd = TimerAction(
        period=30.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', TURTLEBOT3_MODEL,
                    '-file', sdf_path,
                    '-x', x_pose,
                    '-y', y_pose,
                    '-z', z_pose,
                ],
                output='screen',
            )
        ]
    )

    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true',
                                        description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('x_pose', default_value='7.0'))
    ld.add_action(DeclareLaunchArgument('y_pose', default_value='5.0'))
    ld.add_action(DeclareLaunchArgument('z_pose', default_value='0.1'))
    ld.add_action(DeclareLaunchArgument('frame_prefix', default_value=''))

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld