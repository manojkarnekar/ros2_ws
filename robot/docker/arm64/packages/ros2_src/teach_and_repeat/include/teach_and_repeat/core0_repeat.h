#ifndef core0_repeat_H
#define core0_repeat_H

#include <memory>
#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "std_msgs/msg/int16.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "nav2_msgs/action/navigate_through_poses.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class core0R : public rclcpp::Node
{
public:
    core0R() 
    : Node("core0R_node")
    {
        wp_navigation_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", rclcpp::QoS(1).transient_local());
        Nav2GoalPub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(rclcpp::KeepLast(10)));
        NavThPoseAC = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
        flash_pose_sub = this->create_subscription<std_msgs::msg::Int16>("/send_flash_pose", 10, std::bind(&core0R::send_pose_cb, this, _1));
    }

public:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr Nav2GoalPub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr flash_pose_sub;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr NavThPoseAC;
    vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
    geometry_msgs::msg::PoseStamped sendNav2Goal(double x,double y,double z,double ox,double oy,double oz,double ow);
    geometry_msgs::msg::PoseStamped startNav2sendGoal(double x,double y,double z,double ox,double oy,double oz,double ow);
    void updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_);
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
    void resultCallbackNTh(const GoalHandleNavigateThroughPoses::WrappedResult & result);
    void feedbackCallbackNTh(GoalHandleNavigateThroughPoses::SharedPtr,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
    void sendGoal(void);
    void marker_update();
    void core0_repeat();
    void send_pose_cb(const std_msgs::msg::Int16::SharedPtr msg);
    vector<vector<string>> read(std::string read_csv);
    void resetUniqueId();
    int getUniqueId();
    int unique_id {0};
    int time0 = 1000; 
    int send_pose_idx;
    int NumOfPose ;
};

#endif
