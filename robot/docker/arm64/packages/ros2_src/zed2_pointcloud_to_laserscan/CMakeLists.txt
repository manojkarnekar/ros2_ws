cmake_minimum_required(VERSION 3.8)
project(zed2_pointcloud_to_laserscan)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs)
find_package(cv_bridge)


set(dependencies
ament_cmake 
rclcpp
sensor_msgs
cv_bridge
)

install(
  DIRECTORY
    src
  DESTINATION
    share/${PROJECT_NAME}
)

# includes
include_directories(
     include
    ${rclcpp_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

set(_include_dirs "include;/usr/include;/usr/include/opencv4")



# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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


add_executable(zed2_LScan src/zed2_LaserScan.cpp)
ament_target_dependencies(zed2_LScan ${dependencies})

install(TARGETS
  zed2_LScan
  DESTINATION lib/${PROJECT_NAME})
  


