#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// std msg lib
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>

// FollowWaypoints action lib
// #include <nav2_msgs/action/navigate_to_pose.hpp>
// #include <nav2_msgs/action/navigate_through_poses.hpp>
// #include <nav2_msgs/action/follow_waypoints.hpp>

// gemometric msg lib
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// nav_msgs_lib
#include <nav_msgs/msg/odometry.hpp>
// #include <nav2_util/geometry_utils.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

// visualization lib 
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

// sensor msg lib
#include <sensor_msgs/msg/imu.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "sensor_msgs/msg/laser_scan.hpp"

#define RAD2DEG(x) ((x)*180.0/M_PI)
#define DEG2RED(x) ((x)*M_PI/180.0)

#include<iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <chrono>
#include <memory>
#include <math.h>

using namespace std;

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
// using nav2_util::geometry_utils::orientationAroundZAxis;
// using nav2_rviz_plugins::Nav2Panel::startWaypointFollowing;
// using nav2_util::geometry_utils::orientationAroundZAxis;
// #include "grid_map.h"
// #include "commands.h"
// #include "localization.h"


