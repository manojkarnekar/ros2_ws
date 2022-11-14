cmake_minimum_required(VERSION 2.8.3)

project(airbot_model)

find_package(catkin REQUIRED COMPONENTS
  joint_state_publisher
  robot_state_publisher
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2
  geometry_msgs
  xacro
  realsense_gazebo_plugin
  gazebo_ros
)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
