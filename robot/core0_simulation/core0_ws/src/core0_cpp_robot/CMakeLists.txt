cmake_minimum_required(VERSION 3.8)
project(core0_cpp_robot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

ament_python_install_package(src)

install(
  DIRECTORY
    launch urdf worlds meshes config src maps rviz
  DESTINATION
    share/${PROJECT_NAME}
)





if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(xacros)
ament_export_dependencies(urdf)
# ament_package()


add_executable(core0_bot src/core0_bot.cpp)
target_include_directories(core0_bot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(core0_bot PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
add_executable(us_range_pub_sub_sim src/core0_us_sim.cpp)
ament_target_dependencies(us_range_pub_sub_sim rclcpp sensor_msgs std_msgs)

add_executable(us_range_LaserScan_fusion src/us_range_LaserScan_fusion.cpp)
ament_target_dependencies(us_range_LaserScan_fusion rclcpp sensor_msgs std_msgs)

# add_executable(lidar_link_to_us_lidar_link_static_transformation src/lidar_link_to_us_lidar_link_static_transformation.cpp)
# ament_target_dependencies(lidar_link_to_us_lidar_link_static_transformation rclcpp tf2 tf2_ros geometry_msgs )


install(TARGETS 
	core0_bot
	us_range_pub_sub_sim
  us_range_LaserScan_fusion

	DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/command_timeout.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
