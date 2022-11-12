import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_bot')
    default_model_path = os.path.join(pkg_share,'urdf','jetbrain_hotbot.urdf.xacro')
    ekf_param_file = os.path.join(pkg_share,'config','ekf.yaml')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')
    
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path, 
        description='Absolute path to robot urdf file')
    
    start_zed2_full_bed_scan_cmd = Node(
        package='ros2_bot',
        executable='zed2_full_bed_scan',
        name='zed2_full_bed_scan')
    
    start_csv_pose_ros_cmd = Node(
        package='ros2_bot',
        executable='csv_pose_ros',
        name='csv_pose_ros')

    ld = LaunchDescription()
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)

    ld.add_action(start_zed2_full_bed_scan_cmd)
    ld.add_action(start_csv_pose_ros_cmd)
    
    return ld