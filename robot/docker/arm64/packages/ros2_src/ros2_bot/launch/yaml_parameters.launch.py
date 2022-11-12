#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_bot',
            executable='yaml_params_ros2',
            name='yaml_params_ros2',
            parameters=[os.path.join(
                get_package_share_directory('ros2_bot'),
                'config', 'params_demo_ros2.yaml')],
            output='screen'),
    ])
