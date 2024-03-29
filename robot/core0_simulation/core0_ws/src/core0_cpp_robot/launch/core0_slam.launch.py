import os
import xacro
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, ExecuteProcess,RegisterEventHandler, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable, PythonExpression
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_share = FindPackageShare(package='core0_cpp_robot').find('core0_cpp_robot')

     # # Set the path to the RViz configuration settings
    default_rviz_config_path = os.path.join(pkg_share,'rviz/nav2_config.rviz')

    # # Set the path to the URDF file
    # default_urdf_model_path = os.path.join(pkg_share, '/urdf','core0_2022_model.urdf.xacro')

    default_urdf_model_path = os.path.join(pkg_share,'urdf','jetbrain_core0.urdf.xacro')
    # default_urdf_model_path = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("core0_cpp_robot"), "urdf", "jetbrain_core0.urdf.xacro"]
    #         ),
    #     ]
    # )
    default_launch_dir = os.path.join(pkg_share, 'launch')
    robot_localization_file_path = os.path.join(pkg_share, 'config','ekf.yaml')
    # robot_controllers= os.path.join(pkg_share, '/home/sanjeev/sanjeev_ws/src/core0_cpp_robot/config','jetbrain_core0_control.yaml')
    # robot_controllers = PathJoinSubstitution(
    #     ["/home/sanjeev/sanjeev_ws/src/core0_cpp_robot/config",
    #      "jetbrain_core0_control.yaml"],
    # )
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("core0_cpp_robot"),
            "config",
            "jetbrain_core0_control.yaml",
        ]
    )
    robot_name_in_urdf = 'jetbrain_core0'
    world_file_name = 'cafe.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # gazebo_path=SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='/home/sanjeev/sanjeev_ws/install/core0_cpp_robot/share/core0_cpp_robot')
    
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    control =LaunchConfiguration('control')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    model_path = LaunchConfiguration('model_path')
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    namespace = LaunchConfiguration('namespace')
    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel")
                ]
    
    robot_localization_file_path = os.path.join(pkg_share, 'config','ekf.yaml')
    mapping_params_file = os.path.join(pkg_share,'config','mapper_params.yaml')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')
     
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')
    declare_model_path_cmd = DeclareLaunchArgument(
        name='model', 
        default_value=default_urdf_model_path, 
        description='Absolute path to robot urdf file')

    declare_control_cmd = DeclareLaunchArgument(
        name='control', 
        default_value=robot_controllers, 
        description='Absolute path to robot control')


    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        name='use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
    declare_namespace_cmd = DeclareLaunchArgument(
        name='namespace',
        default_value='',
        description='Top-level namespace')

    

      
  # Specify the actions
  
  # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

  # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))


    start_robot_state_publisher_cmd = Node(
        condition=IfCondition(use_robot_state_pub),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
        'robot_description': Command(['xacro ', model])}],
        arguments=[default_urdf_model_path])


    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='jetbrain_core0_spawner',
        arguments=['-entity',
                robot_name_in_urdf,
                '-topic',
                'robot_description',
                '-x','1.0',
                '-y','1.0',
                '-z','0.2',
                '-R','0.0',
                '-P','0.0',
                '-Y','0.0'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
        )

    
   
    spawn_jetbrain_base_controller = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[robot_controllers],
        arguments=['core0_base_controller', '-c', '/controller_manager'],
        output='screen',
    )
    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        parameters=[robot_controllers],
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )
   

   

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[default_urdf_model_path,robot_controllers],
        output="screen",
    )


    spawn_joint_state_publisher = Node(
        condition=UnlessCondition(gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher')

    # A GUI to manipulate the joint state values
    spawn_joint_state_publisher_gui_node = Node(
        condition=IfCondition(gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui')

    
    # start to launch the RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file])
    
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': use_sim_time}])
    
    start_slam_toolbox_cmd = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[mapping_params_file,
          {'use_sim_time': use_sim_time}])
    start_us_pub_sub_cmd = Node(
        package='core0_cpp_robot',
        executable='us_range_pub_sub_sim',
        name='us_range_pub_sub_sim')
  
	# Create the launch description and populate
    ld = LaunchDescription()

      # Declare the launch options
    # ld.add_action(gazebo_path)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_control_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    # ld.add_action()


      # Add any actions
    ld.add_action(spawn_entity)
    ld.add_action(spawn_jetbrain_base_controller)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)    
    ld.add_action(start_robot_state_publisher_cmd)    
    ld.add_action(start_rviz_cmd)
    ld.add_action(spawn_joint_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(control_node)
    ld.add_action(start_robot_localization_cmd)
    ld.add_action(start_slam_toolbox_cmd)
    ld.add_action(start_us_pub_sub_cmd)
      
    return ld
    
   # https://github.com/ros-controls/ros2_control_demos/blob/master/ros2_control_demo_bringup/launch/diffbot.launch.py
    
    
