cmake_minimum_required(VERSION 3.8)
project(qr_code_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
# find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
# find_package(nav2_util REQUIRED)

set(dependencies
ament_cmake 
rclcpp
rclpy
tf2_geometry_msgs
geometry_msgs
tf2_ros
tf2
nav_msgs
sensor_msgs
std_msgs
rclcpp_action
# nav2_msgs
visualization_msgs
# nav2_util
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

add_executable(qr_code_follow src/qr_code_follow.cpp)
ament_target_dependencies(qr_code_follow ${dependencies})

install(TARGETS
  qr_code_follow
  DESTINATION lib/${PROJECT_NAME})
