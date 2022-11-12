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
    pkg_share = get_package_share_directory('base_launch')
    # default_model_path = os.path.join(pkg_share,'urdf','core0_2022_model.urdf')
    ekf_param_file = os.path.join(pkg_share,'config','ekf.yaml')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    model = LaunchConfiguration('model')
    params_file = LaunchConfiguration('params_file')


    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('base_launch'), 'config', 'sar_bot.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

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
        default_value=os.path.join(get_package_share_directory('base_launch'), 'urdf', 'core0_2022_model.urdf'),
        description='Absolute path to robot urdf file')
    
    start_diffTf_cmd = Node(
        package='odometry',
        executable='core0_odom_noise_ros2',
        name='core0_odom_noise_ros2',
        parameters=[params_file],
        output='screen'
        )
    
    # start_motor_control_cmd = Node(
    #     package='control',
    #     executable='core0_control',
    #     name='core0_control',
    #     parameters=[params_file],
    #     output='screen')
    
    # start_imu_cmd = Node(
    #     package='base_launch',
    #     executable='imu_raw',
    #     name='imu_raw_node')

    start_imu_raw_cmd = Node(
        package='imu_raw',
        executable='imu_raw',
        name='imu_raw')
    
    start_complementary_filter_cmd = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node')
    
    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[model])
    
    start_joint_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=remappings,
        arguments=[model])
    

    ld = LaunchDescription()
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(start_diffTf_cmd)
    # ld.add_action(start_motor_control_cmd)
    # ld.add_action(start_imu_cmd)
    ld.add_action(start_imu_raw_cmd)
    ld.add_action(start_complementary_filter_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)

    return ld
