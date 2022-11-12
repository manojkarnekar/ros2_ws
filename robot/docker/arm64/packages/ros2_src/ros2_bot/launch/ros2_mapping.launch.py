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
    mapping_params_file = os.path.join(pkg_share,'params','mapper_params.yaml')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    
    # rplidar_pkg_dir = get_package_share_directory('rplidar_ros')
    # rplidar_launch_file = os.path.join(rplidar_pkg_dir, 'launch', 'rplidar.launch.py')
    
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
    
    # start_rplidar_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(rplidar_launch_file),
    #     launch_arguments={'use_sim_time': use_sim_time}.items())
    
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[default_model_path])

    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_param_file, 
        {'use_sim_time': use_sim_time}])
    
    start_slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapping_params_file,
          {'use_sim_time': use_sim_time}])
    
    start_diffTf_cmd = Node(
        package='ros2_bot',
        executable='diff_tf',
        name='diff_tf_node')
    
    start_motor_control_cmd = Node(
        package='ros2_bot',
        executable='twist_to_motor',
        name='twist_to_motor_node')
    
    start_imu_cmd = Node(
        package='ros2_bot',
        executable='imu_raw',
        name='imu_raw_node')

    
    ld = LaunchDescription()
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_slam_toolbox_cmd)
    # ld.add_action(start_rplidar_cmd)

    ld.add_action(start_diffTf_cmd)
    ld.add_action(start_motor_control_cmd)
    ld.add_action(start_imu_cmd)
    return ld
