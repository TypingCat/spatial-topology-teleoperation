#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('waffle'), 'launch'), '/turtlebot/bringup.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items()),
        Node(
            package='realsense_ros2_camera',
            node_executable='realsense_ros2_camera',
            node_name='realsense_ros2_camera',
            output='screen',
            remappings=[('/tf_static', 'tf_realsense')]),
        Node(
            package='waffle_topology',
            node_executable='waffle_deproject_scan',
            node_name='deproject_scan',
            output='screen'),
    ])
