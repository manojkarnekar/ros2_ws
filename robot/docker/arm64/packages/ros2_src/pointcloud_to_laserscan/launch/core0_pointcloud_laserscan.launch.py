from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/zed2/zed_node/point_cloud/cloud_registered'),
                        ('pclscan','/pcl_scan')],
            parameters=[{
                'target_frame': 'camera_link',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 1.0,
                'angle_min': -0.9,  # -52 deg
                'angle_max': 0.9,  # 52 deg
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.0, 
                'range_min': 0.45,
                'range_max': 1.7, #range = tan(55)*(1.156+0.075)
                'use_inf': True,
                'inf_epsilon': 1.0
                
            }],
            name='pointcloud_to_laserscan'
        )
    ])
