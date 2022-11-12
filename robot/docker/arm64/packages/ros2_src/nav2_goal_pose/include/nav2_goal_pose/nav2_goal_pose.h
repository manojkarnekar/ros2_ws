#ifndef nav2_goal_pose_H
#define nav2_goal_pose_H

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
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>

// gemometric msg lib
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// nav_msgs_lib
#include <nav_msgs/msg/odometry.hpp>
#include <nav2_util/geometry_utils.hpp>
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
using nav2_util::geometry_utils::orientationAroundZAxis;

class Utility : public rclcpp::Node
{
  public:
    explicit Utility() : Node("Utility")
    {
        wp_navigation_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "waypoints", 
            rclcpp::QoS(1).transient_local());

        navigation_action_client_ =
            rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
            this,
            "navigate_to_pose");

        waypoint_follower_action_client_ =
            rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
            this,
            "follow_waypoints");

        nav_through_poses_action_client_ =
            rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(
            this,
            "navigate_through_poses");
        
        navigation_goal_ = nav2_msgs::action::NavigateToPose::Goal();
        waypoint_follower_goal_ = nav2_msgs::action::FollowWaypoints::Goal();
        nav_through_poses_goal_ = nav2_msgs::action::NavigateThroughPoses::Goal();

        target_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_target_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        cv_pose_arr = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/cv_pose", 10, std::bind(&Utility::nav2_goal_pose_callback, this, _1));
        
        lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 
        rclcpp::SensorDataQoS(), std::bind(&Utility::scanCb, this, _1));

        zed2S2pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/zed2_scan", rclcpp::QoS(rclcpp::KeepLast(10)));
        zed2S2BSpub = this->create_publisher<sensor_msgs::msg::LaserScan>("/zed2BothSide_scan", rclcpp::QoS(rclcpp::KeepLast(10)));

    }

  public:
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cv_pose_arr;

    int unique_id {0};
    std::unique_ptr<tf2_ros::TransformBroadcaster> target_tf;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_target_tf;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;

    geometry_msgs::msg::TransformStamped tf_lf, tf_lb, tf_lb_u , tf_lb_d, tf_mm, tf_rf, tf_rb, tf_rb_u, tf_rb_d;

    std::string camera_frame ="camera_link";
    std::string map_frame ="map";
    std::string laser_frame = "laser_link";
    std::string bed_frameA = "bed_linkA";
    std::string bed_frameM = "bed_linkM";
    std::string bed_frameC = "bed_linkC";
    std::string leg_A_frame = "leg_A_frame";
    std::string leg_C_frame = "leg_C_frame";
    std::string bed_frame_mm = "bed_link_mm";
    std::string bed_frame_l = "bed_link_l";
    std::string bed_frame_lf = "bed_link_lf";
    std::string bed_frame_lb = "bed_link_lb";
    std::string bed_frame_lb_u = "bed_link_lb_u";
    std::string bed_frame_lb_d = "bed_link_lb_d";

    std::string bed_frame_r = "bed_link_r";
    std::string bed_frame_rf = "bed_link_rf";
    std::string bed_frame_rb = "bed_link_rb";
    std::string bed_frame_rb_u = "bed_link_rb_u";
    std::string bed_frame_rb_d = "bed_link_rb_d";

    geometry_msgs::msg::TransformStamped tf_sub(std::string parent_frame, std::string child_frame);
    void send_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z);
    void send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z);

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2S2pub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2S2BSpub;

    geometry_msgs::msg::PoseStamped nav2_pose(geometry_msgs::msg::TransformStamped tf_msg);
    void updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_);
    void resetUniqueId();
    int getUniqueId();
    void nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void bed_ai(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    vector<double> Coor_find(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    vector<float> tanrule(float a, float b, float c);
    double Cosin(double a, double b, double c);
    double Sqrt(double x, double y);

    vector<double> minmax_idx(vector<double> v);
    vector<double> cor_lidar_cam(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, double gpx, double gpz);
    vector<double> cor_lidar_cam_tf(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> ranges, std::string parent_frame,  std::string child_frame);
    vector<double> lidar_cluster_l(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> range_angle, double lxt, double lyt, std::string parent_frame, std::string child_frame);
    vector<double> lidar_cluster_r(sensor_msgs::msg::LaserScan::SharedPtr scan_msg, vector<double> range_angle, double lxt, double lyt, std::string parent_frame, std::string child_frame);
    void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses);
    void startNavigation(geometry_msgs::msg::PoseStamped);
    void startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses);

    using NavigationGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
    using WaypointFollowerGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
    using NavThroughPosesGoalHandle =
      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>;
    
    // The NavigateToPose action client
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr 
      navigation_action_client_;
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr
      waypoint_follower_action_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr
      nav_through_poses_action_client_;

    // Goal-related state
    nav2_msgs::action::NavigateToPose::Goal navigation_goal_;
    nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;
    nav2_msgs::action::NavigateThroughPoses::Goal nav_through_poses_goal_;
    NavigationGoalHandle::SharedPtr navigation_goal_handle_;
    WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
    NavThroughPosesGoalHandle::SharedPtr nav_through_poses_goal_handle_;

    rclcpp::Node::SharedPtr client_node_;
    std::chrono::milliseconds server_timeout_;

    double d1, d2, d3, Id, p1, p2, Ip, gama, th1, th2, m, c, m_p, c_p, th_p, k, m_l, m_r, g, 
    gpx, gpy, gpz, wp_x0, wp_z0, del_p, phi_1, phi_2, psi, gpx_l, gpz_l, gpx_r, gpz_r, gpx_m, gpz_m, 
    m_m, gama_m, d_m, p_m, phi_m, psi_m, mic, idx;

    double gamaA, gamaM, gamaC, dm, pa, pm, pc, gpxA, gpzA, gpxM, gpzM, 
    gpxC, gpzC, tha, thc, ipx, l_x, l_y, l_z, dis, range, angle;

    int cnt = 0;
    int maxScanDataIndex,minScanDataIndex,maxBDIndex;
    float ScanPDRatio;


    vector<double> lidar_z, lidar_x;
    vector<float> uf;
    float grid_resolution;
    float bed_length=1.5;
    float pad = 0.50;
    float pady = 0.15;
    unsigned int grid_width, grid_height;
    std::unique_ptr<nav_msgs::msg::OccupancyGrid> grid_;
    vector<float> zed2_to_laser(double rA, double aA, double rC, double aC, double incrAng,  double min_ang, double max_ang);
    vector<float> zed2BS_to_laser(float r_A, float a_A, float r_C, float a_C, float incr_Ang,  float minang, float maxang);
    void Zed2S2RangeScan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2scanpub,float incr, float minlAng, float maxlang);
    double quaternion_to_yaw(geometry_msgs::msg::TransformStamped tf_msg);
};


#endif