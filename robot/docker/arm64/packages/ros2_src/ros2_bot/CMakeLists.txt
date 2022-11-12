cmake_minimum_required(VERSION 3.5)
project(ros2_bot)

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
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)

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
nav2_msgs
visualization_msgs
nav2_util
cv_bridge
)

install(
  DIRECTORY
    launch config urdf maps params src
  DESTINATION
    share/${PROJECT_NAME}
)

# includes
include_directories(
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

ament_export_dependencies(xacros)
ament_export_dependencies(urdf)
ament_package()

add_executable(twist_to_motor src/twist_to_motor.cpp)
ament_target_dependencies(twist_to_motor ${dependencies})

add_executable(diff_tf src/diff_tf.cpp)
ament_target_dependencies(diff_tf ${dependencies})

add_executable(imu_raw src/imu_raw.cpp)
ament_target_dependencies(imu_raw ${dependencies})

add_executable(imu_raw_complemetary src/imu_raw_complemetary.cpp)
ament_target_dependencies(imu_raw_complemetary ${dependencies})

add_executable(lidar_sub_pub src/lidar_sub_pub.cpp)
ament_target_dependencies(lidar_sub_pub ${dependencies})

add_executable(US_range_pub_sub src/us_range_pub_sub.cpp)
ament_target_dependencies(US_range_pub_sub ${dependencies})

add_executable(lidar_cb src/lidar_cb.cpp)
ament_target_dependencies(lidar_cb ${dependencies})

add_executable(costmap_cb src/costmap_cb.cpp)
ament_target_dependencies(costmap_cb ${dependencies})

add_executable(csv_pose_ros src/csv_pose_ros.cpp)
ament_target_dependencies(csv_pose_ros ${dependencies})

add_executable(csv_pose_ros_pcl_scan src/csv_pose_ros_pcl_scan.cpp)
ament_target_dependencies(csv_pose_ros_pcl_scan ${dependencies})

add_executable(nav2_goal_pose src/nav2_goal_pose.cpp)
ament_target_dependencies(nav2_goal_pose ${dependencies})

add_executable(nav2_action_goal_pose src/nav2_action_goal_pose.cpp)
ament_target_dependencies(nav2_action_goal_pose ${dependencies})

add_executable(depth_zed2_scan src/depth_zed2_scan.cpp)
ament_target_dependencies(depth_zed2_scan ${dependencies})

add_executable(zed2_bed_scan src/zed2_bed_scan.cpp)
ament_target_dependencies(zed2_bed_scan ${dependencies})

add_executable(zed2_full_bed_scan src/zed2_full_bed_scan.cpp)
ament_target_dependencies(zed2_full_bed_scan ${dependencies})

# add_executable(zed2_realignment_gp src/nav2_zed2_realignment.cpp)
# ament_target_dependencies(zed2_realignment_gp ${dependencies})

add_executable(us_fusion_ps src/us_fusion_ps.cpp)
ament_target_dependencies(us_fusion_ps ${dependencies})

add_executable(us_fwf_ps src/us_fwf_ps.cpp)
ament_target_dependencies(us_fwf_ps ${dependencies})

add_executable(nav2_send_goal_fb src/nav2_send_goal_fb.cpp)
ament_target_dependencies(nav2_send_goal_fb  ${dependencies})

add_executable(nav2_send_multiple_goal_fb  src/nav2_send_multiple_goal_fb.cpp)
ament_target_dependencies(nav2_send_multiple_goal_fb ${dependencies})

add_executable(nav2_waypoints_follow  src/nav2_waypoints_follow.cpp)
ament_target_dependencies(nav2_waypoints_follow ${dependencies})

add_executable(nav2_waypoints_action src/nav2_waypoints_action.cpp)
ament_target_dependencies(nav2_waypoints_action ${dependencies})

add_executable(nav2_waypoints_action_cv src/nav2_waypoints_action_cv.cpp)
ament_target_dependencies(nav2_waypoints_action_cv ${dependencies})


add_executable(goal_pose_pub src/goal_pose_pub.cpp)
ament_target_dependencies(goal_pose_pub ${dependencies})

add_executable(goal_pose_viz src/goal_pose_viz.cpp)
ament_target_dependencies(goal_pose_viz ${dependencies})

add_executable(point_cloud2_scan src/point_Cloud2_scan.cpp)
ament_target_dependencies(point_cloud2_scan ${dependencies})

add_executable(AFS_IMU_raw_node src/afs_imu.cpp)
ament_target_dependencies(AFS_IMU_raw_node ${dependencies})

add_executable(AfsOdom_node src/afs_odometry.cpp)
ament_target_dependencies(AfsOdom_node ${dependencies})

add_executable(AfsControl_node src/afs_tricycle_control.cpp)
ament_target_dependencies(AfsControl_node ${dependencies})

add_executable(core0TR_node src/core0_teach&repeat.cpp)
ament_target_dependencies(core0TR_node ${dependencies})

add_executable(core0R_node src/core0_repeat.cpp)
ament_target_dependencies(core0R_node ${dependencies})

add_executable(core0T_node src/core0_teach.cpp)
ament_target_dependencies(core0T_node ${dependencies})

add_executable(core0FP_node src/flash_pose.cpp)
ament_target_dependencies(core0FP_node ${dependencies})

add_executable(yaml_params_ros2 src/yaml_params_ros2.cpp)
ament_target_dependencies(yaml_params_ros2 ${dependencies})

add_executable(zed2_LScan src/zed2_LaserScan.cpp)
ament_target_dependencies(zed2_LScan ${dependencies})



install(TARGETS
  twist_to_motor
  diff_tf
  imu_raw
  imu_raw_complemetary
  lidar_sub_pub
  US_range_pub_sub
  lidar_cb
  costmap_cb
  csv_pose_ros
  csv_pose_ros_pcl_scan
  nav2_action_goal_pose
  nav2_send_goal_fb
  nav2_send_multiple_goal_fb
  nav2_waypoints_follow
  nav2_waypoints_action
  nav2_waypoints_action_cv
  goal_pose_pub
  goal_pose_viz
  point_cloud2_scan
  zed2_bed_scan
  zed2_full_bed_scan
  # zed2_realignment_gp
  us_fusion_ps
  us_fwf_ps
  AFS_IMU_raw_node
  AfsOdom_node
  AfsControl_node
  core0TR_node
  core0R_node
  core0T_node
  core0FP_node
  yaml_params_ros2
  zed2_LScan
  DESTINATION lib/${PROJECT_NAME})
