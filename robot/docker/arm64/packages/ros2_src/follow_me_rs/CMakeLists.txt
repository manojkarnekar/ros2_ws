cmake_minimum_required(VERSION 3.8)
project(follow_me_rs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV 4 REQUIRED)  #4
find_library(ZBAR_LIBRARIES NAMES zbar)

set(dependencies
ament_cmake
rclcpp
std_msgs
sensor_msgs
nav_msgs
# Eigen3
)

# include_directories(
#   include
# )

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(_include_dirs "include;/usr/include;/usr/include/opencv4")

add_executable(qr_code_pose src/qr_code_pose.cpp) 
ament_target_dependencies(qr_code_pose ${dependencies})
target_link_libraries(qr_code_pose zbar ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

install(TARGETS
  qr_code_pose
  DESTINATION lib/${PROJECT_NAME})

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
