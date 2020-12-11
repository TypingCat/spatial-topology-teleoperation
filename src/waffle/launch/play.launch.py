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
    rviz_config = LaunchConfiguration(
        'rviz_config',
        default=os.path.join(get_package_share_directory('waffle'), 'config', 'play.rviz'))

    return LaunchDescription([
        Node(
            package='rviz2',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
        Node(
            package='image_transport',
            node_executable='republish',
            node_name='image_raw_transport',
            arguments=['compressed', 'raw'],
            output='screen',
            remappings=[
                ('/in/compressed', '/camera/color/image_raw/compressed'),
                ('/out', '/camera/color/image_raw/uncompressed')]),
        Node(
            package='image_transport',
            node_executable='republish',
            node_name='image_segmentation_transport',
            arguments=['compressed', 'raw'],
            output='screen',
            remappings=[
                ('/in/compressed', '/camera/color/segmentation/compressed'),
                ('/out', '/camera/color/segmentation/uncompressed')]),
    ])
