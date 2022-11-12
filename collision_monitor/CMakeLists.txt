cmake_minimum_required(VERSION 3.8)
project(collision_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs)
find_package(std_msgs)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(dependencies
ament_cmake 
rclcpp
geometry_msgs
sensor_msgs
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

add_executable(collision_M src/collision_monitor.cpp)
ament_target_dependencies(collision_M ${dependencies})

install(TARGETS
collision_M
  DESTINATION lib/${PROJECT_NAME})

ament_package()
