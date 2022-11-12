#include <memory>
#include <chrono>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "nav2_msgs/action/navigate_through_poses.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class core0TR : public rclcpp::Node
{
public:
    core0TR() 
    : Node("core0TR_node")
    {
        wp_navigation_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", rclcpp::QoS(1).transient_local());
        GoalPosePub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", rclcpp::QoS(rclcpp::KeepLast(10)));
        NavThPoseAC = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
        core0_repeat();
    
 
    }

public:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr GoalPosePub;
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr NavThPoseAC;
    
    vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
    
    geometry_msgs::msg::PoseStamped sendNav2Goal(double x,double y,double z,double ox,double oy,double oz,double ow);
    void updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_);

    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
    
    void resultCallbackNTh(const GoalHandleNavigateThroughPoses::WrappedResult & result);
    
    void feedbackCallbackNTh(GoalHandleNavigateThroughPoses::SharedPtr,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback);
  
    void feedback_time();
    
    void core0_repeat();
    void sendGoal(void);
    void resetUniqueId();
    int getUniqueId();
    int unique_id {0};
    vector<float> pose1{-1.0,0.0,0.0,0.0,0.0,3.14,1.0};
    vector<float> pose2{-1.2,0.0,0.0,0.0,0.0,3.14,1.0};
    vector<float> pose3{-1.5,0.0,0.0,0.0,0.0,3.14,1.0};
    vector<float> pose4{-1.8,0.0,0.0,0.0,0.0,3.14,1.0};
    vector<float> pose5{-2.2,0.0,0.0,0.0,0.0,3.14,1.0};
    int time0 = 10*1000;
    


};


void core0TR::sendGoal(void) 
{
    while (!NavThPoseAC->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }
    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses.push_back(sendNav2Goal(pose1.at(0),pose1.at(1),pose1.at(2),pose1.at(3),pose1.at(4),pose1.at(5),pose1.at(6)));
    goal_msg.poses.push_back(sendNav2Goal(pose2.at(0),pose2.at(1),pose2.at(2),pose2.at(3),pose2.at(4),pose2.at(5),pose2.at(6)));
    goal_msg.poses.push_back(sendNav2Goal(pose3.at(0),pose3.at(1),pose3.at(2),pose3.at(3),pose3.at(4),pose3.at(5),pose3.at(6)));
    goal_msg.poses.push_back(sendNav2Goal(pose4.at(0),pose4.at(1),pose4.at(2),pose4.at(3),pose4.at(4),pose4.at(5),pose4.at(6)));
    goal_msg.poses.push_back(sendNav2Goal(pose5.at(0),pose5.at(1),pose5.at(2),pose5.at(3),pose5.at(4),pose5.at(5),pose5.at(6)));
    
    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&core0TR::feedbackCallbackNTh, this, _1, _2);
    send_goal_options.result_callback = std::bind(&core0TR::resultCallbackNTh, this, _1);
    NavThPoseAC->async_send_goal(goal_msg, send_goal_options);
}




void core0TR::core0_repeat()
{
    cout << "core0_repeat"<<endl;
    
    acummulated_poses_.push_back(sendNav2Goal(pose1.at(0),pose1.at(1),pose1.at(2),pose1.at(3),pose1.at(4),pose1.at(5),pose1.at(6)));
    acummulated_poses_.push_back(sendNav2Goal(pose2.at(0),pose2.at(1),pose2.at(2),pose2.at(3),pose2.at(4),pose2.at(5),pose2.at(6)));
    acummulated_poses_.push_back(sendNav2Goal(pose3.at(0),pose3.at(1),pose3.at(2),pose3.at(3),pose3.at(4),pose3.at(5),pose3.at(6)));
    acummulated_poses_.push_back(sendNav2Goal(pose4.at(0),pose4.at(1),pose4.at(2),pose4.at(3),pose4.at(4),pose4.at(5),pose4.at(6)));
    acummulated_poses_.push_back(sendNav2Goal(pose5.at(0),pose5.at(1),pose5.at(2),pose5.at(3),pose5.at(4),pose5.at(5),pose5.at(6)));
    updateWpNavigationMarkers(acummulated_poses_);
   
    acummulated_poses_.clear();

    // rclcpp::sleep_for(std::chrono::milliseconds(time0));
    // exit(0);
}


void core0TR::feedbackCallbackNTh(GoalHandleNavigateThroughPoses::SharedPtr,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
  }


void core0TR::resultCallbackNTh(const GoalHandleNavigateThroughPoses::WrappedResult & result)
  {
    switch (result.code) {
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

void core0TR::updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_)
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

void core0TR::resetUniqueId()
{
  unique_id = 0;
}

int core0TR::getUniqueId()
{
  int temp_id = unique_id;
  unique_id += 1;
  return temp_id;
}


geometry_msgs::msg::PoseStamped core0TR::sendNav2Goal(double x,double y,double z,double ox,double oy,double oz,double ow)
{
    cout << "sendNav2Goal"<<endl;
    geometry_msgs::msg::PoseStamped goal_msg;
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = "map";
    goal_msg.pose.position.x = x;
    goal_msg.pose.position.y = y;
    goal_msg.pose.position.z = z;
    goal_msg.pose.orientation.x = ox;
    goal_msg.pose.orientation.y = oy;
    goal_msg.pose.orientation.z = oz;
    goal_msg.pose.orientation.w = ow;
    GoalPosePub->publish(goal_msg);
    return goal_msg;
}




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<core0TR>();
  node->core0_repeat();
  // node->sendGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

