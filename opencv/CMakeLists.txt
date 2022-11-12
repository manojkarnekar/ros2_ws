cmake_minimum_required(VERSION 3.8)
project(opencv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(urdf REQUIRED)
# find_package(xacro REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV 4 REQUIRED)

set(dependencies
ament_cmake 
rclcpp
rclpy
geometry_msgs
std_msgs
)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

set(_include_dirs "include;/usr/include;/usr/include/opencv4")

add_executable(read_write src/read_write.cpp)
ament_target_dependencies(read_write ${dependencies})
target_link_libraries(read_write ${OpenCV_LIBRARIES})

add_executable(video src/video.cpp)
ament_target_dependencies(video ${dependencies})
target_link_libraries(video ${OpenCV_LIBRARIES})

add_executable(image_pro src/image_pro.cpp)
ament_target_dependencies(image_pro ${dependencies})
target_link_libraries(image_pro ${OpenCV_LIBRARIES})

add_executable(crop src/crop.cpp)
ament_target_dependencies(crop ${dependencies})
target_link_libraries(crop ${OpenCV_LIBRARIES})

add_executable(annotating src/annotating.cpp)
ament_target_dependencies(annotating ${dependencies})
target_link_libraries(annotating ${OpenCV_LIBRARIES})

add_executable(threshold src/threshold.cpp)
ament_target_dependencies(threshold ${dependencies})
target_link_libraries(threshold ${OpenCV_LIBRARIES})

add_executable(blob src/blob.cpp)
ament_target_dependencies(blob ${dependencies})
target_link_libraries(blob ${OpenCV_LIBRARIES})

add_executable(edge_detection src/edge.cpp)
ament_target_dependencies(edge_detection ${dependencies})
target_link_libraries(edge_detection ${OpenCV_LIBRARIES})

install(TARGETS
  read_write
  video
  image_pro
  crop
  annotating
  threshold
  blob
  edge_detection
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
