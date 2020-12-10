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
