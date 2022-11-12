cmake_minimum_required(VERSION 3.8)
project(teach_and_repeat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(sensor_msgs)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

set(dependencies
ament_cmake 
rclcpp
tf2_geometry_msgs
geometry_msgs
tf2_ros
visualization_msgs
tf2
nav2_msgs
sensor_msgs
rclcpp_action
std_msgs
)


install(
  DIRECTORY
    src
  DESTINATION
    share/${PROJECT_NAME})


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

add_executable(core0R_node src/core0_repeat.cpp)
ament_target_dependencies(core0R_node ${dependencies})

add_executable(core0T_node src/core0_teach.cpp)
ament_target_dependencies(core0T_node ${dependencies})

install(TARGETS
  core0R_node
  core0T_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
