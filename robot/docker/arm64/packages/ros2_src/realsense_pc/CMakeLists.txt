cmake_minimum_required(VERSION 3.8)
project(realsense_pc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
# find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(OpenCV 4 REQUIRED) 
find_library(ZBAR_LIBRARIES NAMES zbar)

set(dependencies
ament_cmake
rclcpp
std_msgs
sensor_msgs
Eigen3
tf2_geometry_msgs
geometry_msgs
tf2_ros
tf2
)


include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

set(_include_dirs "include;/usr/include;/usr/include/opencv4")


add_executable(depth_pc src/depth_pc.cpp)    
ament_target_dependencies(depth_pc ${dependencies})
target_link_libraries(depth_pc zbar ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

add_executable(qr_scan src/qr_scan.cpp)
ament_target_dependencies(qr_scan ${dependencies})
target_link_libraries(qr_scan zbar ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

add_executable(image_rs src/image_realsense.cpp)
ament_target_dependencies(image_rs ${dependencies})
target_link_libraries(image_rs zbar ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})

add_executable(cv_localization src/cv_localization.cpp)
ament_target_dependencies(cv_localization ${dependencies})
target_link_libraries(cv_localization zbar ${OpenCV_LIBRARIES} ${realsense2_LIBRARY})


install(TARGETS
  depth_pc
  qr_scan
  image_rs
  cv_localization
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
