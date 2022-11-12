from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            name='rplidar_composition',
            package='rplidar_ros',
            executable='rplidar_composition',
            output='screen',
            parameters=[{
                'serial_port': '/dev/rplidars2',
                'serial_baudrate': 1000000,
                'frame_id': 'laser_link',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ])
