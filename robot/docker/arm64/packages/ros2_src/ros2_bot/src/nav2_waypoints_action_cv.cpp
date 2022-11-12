#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
// FollowWaypoints action lib
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/geometry_utils.hpp"
#include <std_msgs/msg/bool.hpp>
#define RAD2DEG(x) ((x)*180.0/M_PI)
#define DEG2RED(x) ((x)*M_PI/180.0)
// #include <visualization_msgs/msg/marker.hpp>

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

class Nav2Client : public rclcpp::Node
{
public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;
  
  

  rclcpp_action::Client<FollowWaypoints>::SharedPtr waypoints_client_ptr_;

  explicit Nav2Client(): Node("nav2_waypoints_action_cv")
  {
    this->waypoints_client_ptr_  = rclcpp_action::create_client<FollowWaypoints>(this, "follow_waypoints");

    this->declare_parameter<std::string>("parent_frame", "map");
    this->get_parameter("parent_frame", parent_frame_);

    this->declare_parameter<std::string>("child_frame", "base_footprint");
    this->get_parameter("child_frame", child_frame_);

    cv_pose_arr = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/cv_pose", 10, std::bind(&Nav2Client::nav2_goal_pose_callback, this, _1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 
    rclcpp::SystemDefaultsQoS(), 
    std::bind(&Nav2Client::odomCallback, this, std::placeholders::_1));

    nav2_cancel_goal_sub = this->create_subscription<std_msgs::msg::Bool>("nav2_cancel_goal", 
    rclcpp::SystemDefaultsQoS(), 
    std::bind(&Nav2Client::nav2_cancel_goal_cb, this, std::placeholders::_1));
    
    pose_stamped_pub_L = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_l", 10);
    pose_stamped_pub_L2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_l2", 10);

    pose_stamped_pub_M = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_m", 10);

    pose_stamped_pub_R = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_r", 10);
    pose_stamped_pub_R2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_r2", 10);

    wp_navigation_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", rclcpp::QoS(1).transient_local());

  }
  
private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_L, pose_stamped_pub_L2, pose_stamped_pub_M, pose_stamped_pub_R, pose_stamped_pub_R2;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cv_pose_arr;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr nav2_cancel_goal_sub;

    void nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);
    void nav2_cancel_goal_cb(std_msgs::msg::Bool::SharedPtr msg);
    void viz_pose_stamp(geometry_msgs::msg::PoseStamped goal_poses,rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub);

    float x,y,d;
    float d_i = 0.0;
    float d_f = 0.0;
    float d_t = 0.55;
    int cnt = 0;

    float x_l, y_l, x_m, y_m, x_r, y_r, T_x, T_y, O_z, O_w;
    std::string parent_frame_;
    std::string child_frame_;
    std::string odom_topic = "/odometry/filtered";
    float odom_tx, odom_ty, odom_az, odom_aw, l_p, m_p, r_p, phi_l, phi_r, p, h, phi;
    geometry_msgs::msg::PoseStamped left_pose, left_pose2, mid_pose, right_pose, right_pose2, left_pose_T, right_pose_T;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
    void updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_);
    void resetUniqueId();
    int getUniqueId();
    int unique_id {0};
    void sendGoal(std::vector<geometry_msgs::msg::PoseStamped> poses_);
    void feedbackCallback(GoalHandleFollowWaypoints::SharedPtr,const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void resultCallback(const GoalHandleFollowWaypoints::WrappedResult & result);
    geometry_msgs::msg::PoseStamped nav2_pose(float t_x, float t_y, double theta);
    void Nav2_Cancel_goal();
    GoalHandleFollowWaypoints::SharedPtr waypoint_follower_goal_handle_;
    std::chrono::milliseconds server_timeout_;
    rclcpp::Node::SharedPtr client_node_;
    float del_r = 0.7, p1, p2, d1, d2, d3, th1, th2, i_d, th_i_d, gpx, gpy, gpth;
    vector<float> realignment(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

};


void Nav2Client::viz_pose_stamp(geometry_msgs::msg::PoseStamped goal_poses,rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub)
{
    pose_stamped_pub->publish(goal_poses);
}

geometry_msgs::msg::PoseStamped Nav2Client::nav2_pose(float t_x, float t_y, double theta)
{   

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = t_x;
    goal_pose.pose.position.y = t_y;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation = orientationAroundZAxis(theta);
    // goal_pose.pose.orientation.x = 0.0;
    // goal_pose.pose.orientation.y = 0.0;
    // goal_pose.pose.orientation.z = o_z;
    // goal_pose.pose.orientation.w = o_w;
    return goal_pose;

}

void Nav2Client::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    auto current_time = rclcpp::Time(msg->header.stamp);
    odom_tx = msg->pose.pose.position.x;
    odom_ty = msg->pose.pose.position.y;
    odom_az = msg->pose.pose.orientation.z;
    odom_aw = msg->pose.pose.orientation.w;
    // cout<<"T_x = " <<odom_tx<<" "<<"T_y = " <<odom_ty<<" "<<"O_z = " << odom_az<< " O_w = " << odom_aw<< endl;
}

vector<float> Nav2Client::realignment(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    p1 = hypot((msg->data.at(2)-msg->data.at(0)), (msg->data.at(3)-msg->data.at(1)));
    p2 = hypot((msg->data.at(5)-msg->data.at(3)), (msg->data.at(4)-msg->data.at(2)));

    d1 = hypot(msg->data.at(0), msg->data.at(1));
    d2 = hypot(msg->data.at(2), msg->data.at(3));
    d3 = hypot(msg->data.at(4), msg->data.at(5));

    i_d=abs(msg->data.at(9)*1000);

    th1 = acos(((d1*d1)+(d2*d2)-(p1*p1))/(2*d1*d2));
    th2 = acos(((d2*d2)+(d3*d3)-(p2*p2))/(2*d2*d3));

    th_i_d=(i_d*0.05729)*(0.01745);

    if (msg->data.at(9)>0)
    {
      gpx=(d2*sin(1.57-th_i_d)-2);
      gpy=-d2*cos(1.57-th_i_d);
      gpth=-th_i_d*(57.295);

    }

    else
    {
      gpx=(d2*sin(1.57-th_i_d)-2);
      gpy=d2*cos(1.57-th_i_d);
      gpth=th_i_d*(57.295);
    }
    vector<float> pose;
    pose.push_back(gpx);
    pose.push_back(gpy);
    pose.push_back(gpth);
    return pose;

}
void Nav2Client::nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  vector<float> ini_poses = realignment(msg);

    // d_i = msg->data;
    // l_p = msg->data.at(6);
    // m_p = msg->data.at(7);
    // r_p = msg->data.at(8);


    // p = msg->data.at(9);
    // h = m_p;

    // if(h != 0.0 && p!=0)
    // {
    //   phi = asin(-p/h);
    // }

    // RCLCPP_INFO(get_logger(), "phi--->{%f}", RAD2DEG(phi));



    // x_m = msg->data.at(7);
    // y_m = x_m * tan((phi));

    // x_l = x_m;
    // y_l = -(msg->data.at(6)+ y_m + del_r);

    // x_r = x_m;
    // y_r =  (msg->data.at(8)+ y_m + del_r);

    // mid_pose = nav2_pose(x_m-1.0, y_m, 0.0);

    // right_pose = nav2_pose(x_r, y_r, 0.0);
    // right_pose2 = nav2_pose(x_r+1, y_r, 0.0);
    // right_pose_T = nav2_pose(x_r, y_r, 179.0);

    // left_pose = nav2_pose(x_l, y_l, 0.0);
    // left_pose2 = nav2_pose(x_l+1, y_l, 0.0);
    // left_pose_T = nav2_pose(x_l, y_l, 179.0);
    
    std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
    acummulated_poses_.push_back(nav2_pose(ini_poses.at(0), ini_poses.at(1), ini_poses.at(2)));

    // if(RAD2DEG(phi) > 0)
    // {
    //   acummulated_poses_.push_back(nav2_pose(0.0, 0.0, -RAD2DEG(phi)));
    // }


    // if(RAD2DEG(phi) < 0)
    // {
    //   acummulated_poses_.push_back(nav2_pose(0.0, 0.0, RAD2DEG(phi)));
    // }

    
    // acummulated_poses_.push_back(mid_pose);
    // acummulated_poses_.push_back(right_pose);
    // acummulated_poses_.push_back(right_pose2);
    // acummulated_poses_.push_back(right_pose_T);

    // acummulated_poses_.push_back(nav2_pose(x_m-2.0, y_m, 0.0));
    // acummulated_poses_.push_back(mid_pose);

    // acummulated_poses_.push_back(left_pose);
    // acummulated_poses_.push_back(left_pose2);
    // acummulated_poses_.push_back(left_pose_T);
    // acummulated_poses_.push_back(nav2_pose(x_m-2.0, y_m, 0.0));

    updateWpNavigationMarkers(acummulated_poses_);
    
    if(cnt<1)
    {
        d = (d_i-d_t);
        // updateWpNavigationMarkers(acummulated_poses_);
        // sendGoal(acummulated_poses_);
    }
    cnt +=1;

    acummulated_poses_.clear();

}

void Nav2Client::updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_)
{
    resetUniqueId();
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    for (size_t i = 0; i < poses_.size(); i++)
    {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header = poses_[i].header;
        arrow_marker.id = getUniqueId();
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.pose = poses_[i].pose;
        arrow_marker.scale.x = 0.3;
        arrow_marker.scale.y = 0.05;
        arrow_marker.scale.z = 0.02;
        arrow_marker.color.r = 0;
        arrow_marker.color.g = 255;
        arrow_marker.color.b = 0;
        arrow_marker.color.a = 1.0f;
        arrow_marker.lifetime = rclcpp::Duration(0s);
        arrow_marker.frame_locked = false;
        marker_array->markers.push_back(arrow_marker);

        // Draw a red circle at the waypoint pose
        visualization_msgs::msg::Marker circle_marker;
        circle_marker.header = poses_[i].header;
        circle_marker.id = getUniqueId();
        circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;
        circle_marker.pose = poses_[i].pose;
        circle_marker.scale.x = 0.05;
        circle_marker.scale.y = 0.05;
        circle_marker.scale.z = 0.05;
        circle_marker.color.r = 255;
        circle_marker.color.g = 0;
        circle_marker.color.b = 0;
        circle_marker.color.a = 1.0f;
        circle_marker.lifetime = rclcpp::Duration(0s);
        circle_marker.frame_locked = false;
        marker_array->markers.push_back(circle_marker);

        // Draw the waypoint number
        visualization_msgs::msg::Marker marker_text;
        marker_text.header = poses_[i].header;
        marker_text.id = getUniqueId();
        marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::msg::Marker::ADD;
        marker_text.pose = poses_[i].pose;
        marker_text.pose.position.z += 0.2;  // draw it on top of the waypoint
        marker_text.scale.x = 0.07;
        marker_text.scale.y = 0.07;
        marker_text.scale.z = 0.07;
        marker_text.color.r = 0;
        marker_text.color.g = 255;
        marker_text.color.b = 0;
        marker_text.color.a = 1.0f;
        marker_text.lifetime = rclcpp::Duration(0s);
        marker_text.frame_locked = false;
        marker_text.text = "wp_" + std::to_string(i + 1);
        marker_array->markers.push_back(marker_text);
    }

    if (marker_array->markers.empty()) 
    {
    visualization_msgs::msg::Marker clear_all_marker;
    clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(clear_all_marker);
    }

    wp_navigation_markers_pub_->publish(std::move(marker_array));
    // poses_.clear();

}

void Nav2Client::resetUniqueId()
{
  unique_id = 0;
}

int Nav2Client::getUniqueId()
{
  int temp_id = unique_id;
  unique_id += 1;
  return temp_id;
}

void Nav2Client::sendGoal(std::vector<geometry_msgs::msg::PoseStamped> poses_) 
{
    while (!this->waypoints_client_ptr_->wait_for_action_server()) 
    {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }
    
    auto goal_msg = FollowWaypoints::Goal();
    goal_msg.poses = poses_;
    

    auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Nav2Client::resultCallback, this, _1);

    waypoints_client_ptr_->async_send_goal(goal_msg, send_goal_options);
    poses_.clear();
}

//feedback
void Nav2Client::feedbackCallback(GoalHandleFollowWaypoints::SharedPtr,const std::shared_ptr<const FollowWaypoints::Feedback> feedback)
{
  RCLCPP_INFO(get_logger(), "current_waypoint = %d", feedback->current_waypoint);
}

//result
void Nav2Client::resultCallback(const GoalHandleFollowWaypoints::WrappedResult & result)
{
  switch (result.code) 
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "Success!!!");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(get_logger(), "Unknown result code");
      return;
  }
}

void Nav2Client::nav2_cancel_goal_cb(std_msgs::msg::Bool::SharedPtr msg)
{
  if(msg->data)
  {
    RCLCPP_INFO(get_logger(), "Nav2_Cancel_goal!!!");
    // std::cout<<"Nav2_Cancel_goal"<<std::endl;
    Nav2_Cancel_goal();
  }
  // RCLCPP_INFO(msg->data ? "true" : "false");
}

void Nav2Client::Nav2_Cancel_goal()
{

  if (waypoint_follower_goal_handle_) {
    auto future_cancel =
      waypoints_client_ptr_->async_cancel_goal(waypoint_follower_goal_handle_);

    if (rclcpp::spin_until_future_complete(client_node_, future_cancel, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(client_node_->get_logger(), "Failed to cancel waypoint follower");
    } 
    else 
    {
      waypoint_follower_goal_handle_.reset();
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
//   node->sendGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}