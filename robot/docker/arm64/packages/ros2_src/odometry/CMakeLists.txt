cmake_minimum_required(VERSION 3.8)
project(odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

set(dependencies
ament_cmake
rclcpp
std_msgs
geometry_msgs
nav_msgs
tf2
tf2_ros)

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

include_directories(
  include
)

ament_package()

add_executable(core0_odom src/core0_odom.cpp)
ament_target_dependencies(core0_odom ${dependencies})

add_executable(afs_odom src/afs_odom.cpp)
ament_target_dependencies(afs_odom ${dependencies})

add_executable(core0_odom_noise_ros2 src/core0_odom_noise_ros2.cpp)
ament_target_dependencies(core0_odom_noise_ros2 ${dependencies})

add_executable(diff_tf src/diff_tf.cpp)
ament_target_dependencies(diff_tf ${dependencies})


install(TARGETS
  core0_odom
  afs_odom
  core0_odom_noise_ros2
  diff_tf
  DESTINATION lib/${PROJECT_NAME})