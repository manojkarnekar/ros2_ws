cmake_minimum_required(VERSION 3.8)
project(nav2_goal_pose)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)

set(dependencies
ament_cmake 
rclcpp
tf2_geometry_msgs
geometry_msgs
tf2_ros
tf2
nav_msgs
sensor_msgs
std_msgs
rclcpp_action
nav2_msgs
visualization_msgs
nav2_util
)

install(
  DIRECTORY
    src
  DESTINATION
    share/${PROJECT_NAME}
)

include_directories(
  include
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

ament_package()

add_executable(nav2_fwp 
src/nav2_camera.cpp 
src/nav2_goal_pose.cpp 
src/nav2_lidar.cpp 
src/nav2_math.cpp 
src/nav2_rviz.cpp 
src/nav2_tf.cpp 
src/nav2_wp.cpp
src/nav2_main.cpp)

ament_target_dependencies(nav2_fwp ${dependencies})

install(TARGETS
  nav2_fwp
  DESTINATION lib/${PROJECT_NAME})