
import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, GroupAction
from launch.substitutions import LaunchConfiguration, FindExecutable, Command, PathJoinSubstitution
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ARGUMENTS = [
    DeclareLaunchArgument('world_path', default_value='/home/himanshu/test_ws/src/jetbrain_hotbot/worlds/smalltown.world',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():


    use_slam = LaunchConfiguration('slam', default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


    # launch args
    world_path = LaunchConfiguration('world_path')

    # get urdf via Xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("jetbrain_hotbot"), "urdf", "jetbrain_hotbot.urdf.xacro"]
            ),
            " is_sim:=true"
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    config_jetbrain_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("jetbrain_hotbot"),
         "config",
         "jetbrain_hotbot_control.yaml"],
    )

    robot_localization_file_path = os.path.join('/home/himanshu/test_ws/src/robot_localization/params', 'params/ekf.yaml') 

    spawn_jetbrain_base_controller = Node(
        package='controller_manager',
        executable='spawner.py',
        parameters=[config_jetbrain_velocity_controller],
        arguments=['hotbot_base_controller', '-c', '/controller_manager'],
        output='screen',
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': True}, robot_description],
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner.py',
        arguments=['joint_state_broadcaster', '-c', '/controller_manager'],
        output='screen',
    )

    # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_joint_state_broadcaster,
            on_exit=[spawn_jetbrain_base_controller],
        )
    )
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )

    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_husky',
        arguments=['-entity',
                   'jetbrain_hotbot',
                   '-topic',
                   'robot_description'],
        output='screen',
    )

    # footprint_publisher = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     output='screen',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    # )
    # static_tf_nodes = GroupAction([

    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_transform_publisher',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'laser_link']
    #     ),
    # ])

    
    #spawn_rviz2
    # spawn_rviz2 = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', str(os.path.join('/home/himanshu/test_ws/src/jetbrain_hotbot/config', 'config/display.rivz'))]
    # )


    # slam_toolbox = Node(
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     package='slam_toolbox',
    #     executable='async_slam_toolbox_node',
    #     name='slam_toolbox',
    #     output='screen',
    #     condition=launch.conditions.IfCondition(use_slam)
    # )

    # Start robot localization using an Extended Kalman filter
    start_robot_localization_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[robot_localization_file_path, 
        {'use_sim_time': True}])

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(spawn_joint_state_broadcaster)
    ld.add_action(diffdrive_controller_spawn_callback)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_robot)
    # ld.add_action(spawn_rviz2)
    # ld.add_action(footprint_publisher)
    # ld.add_action(static_tf_nodes)
    # ld.add_action(slam_toolbox)
    ld.add_action(start_robot_localization_cmd)

    return ld
