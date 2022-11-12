#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

def generate_launch_description():
    share_dir = get_package_share_directory('ls01b_v2')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'ls01b_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                           share_dir, 'params', 'ls01b_v2.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='ls01b_v2',
                                node_executable='ls01b_node',
                                node_name='ls01b_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                node_namespace='/',
                                )
         
    return LaunchDescription([
        params_declare,
        driver_node,
    ])
