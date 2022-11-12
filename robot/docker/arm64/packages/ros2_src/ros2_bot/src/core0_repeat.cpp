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

void core0R::marker_update()
{
    updateWpNavigationMarkers(acummulated_poses_);
    acummulated_poses_.clear();
}
void core0R::send_pose_cb(const std_msgs::msg::Int16::SharedPtr msg)
{

  send_pose_idx = msg->data;
  if (0 < send_pose_idx && send_pose_idx < NumOfPose)
  {
    sendGoal();
  }
}

void core0R::sendGoal(void) 
{
    while (!NavThPoseAC->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }
    auto goal_msg = NavigateThroughPoses::Goal();

    vector<vector<string>> pose = read("core0_teach.csv");
    vector<vector<string>> goal_pose_idx = read("core0_flash_pose.csv");
    NumOfPose = goal_pose_idx.size();

    if (send_pose_idx == 0)
    {
      startNav2sendGoal(stof(pose[1][0]),stof(pose[1][1]), stof(pose[1][2]), stof(pose[1][3]), stof(pose[1][4]), stof(pose[1][5]), stof(pose[1][6]));
    }
    

    if (send_pose_idx == 1)
    {
      cout<<"goal_pose_idx[send_pose_idx][0];"<<goal_pose_idx[send_pose_idx][0]<<endl;
      for (int i = 1; i <= stof(goal_pose_idx[send_pose_idx][0]); i+=1)
      {
        goal_msg.poses.push_back(sendNav2Goal(stof(pose[i][0]),stof(pose[i][1]), stof(pose[i][2]), stof(pose[i][3]), stof(pose[i][4]), stof(pose[i][5]), stof(pose[i][6])));
      }
      auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
      send_goal_options.feedback_callback = std::bind(&core0R::feedbackCallbackNTh, this, _1, _2);
      send_goal_options.result_callback = std::bind(&core0R::resultCallbackNTh, this, _1);
      NavThPoseAC->async_send_goal(goal_msg, send_goal_options);
    }

    if (send_pose_idx > 1)
    {
      cout<<"goal_pose_idx[send_pose_idx][0] = "<<goal_pose_idx[send_pose_idx][0]<<endl;
      int prev_pose = send_pose_idx -1;
      cout<<"prev_pose = "<<prev_pose<<endl;
      int start_pose = stof(goal_pose_idx[prev_pose][0]);
      cout<<"start_pose == "<<start_pose<<endl;
      if (start_pose==0)
      {
        start_pose=start_pose+1;
      }


      for (int i = start_pose; i <= stof(goal_pose_idx[send_pose_idx][0]); i+=1)
      {
        goal_msg.poses.push_back(sendNav2Goal(stof(pose[i][0]),stof(pose[i][1]), stof(pose[i][2]), stof(pose[i][3]), stof(pose[i][4]), stof(pose[i][5]), stof(pose[i][6])));
      }
      auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
      send_goal_options.feedback_callback = std::bind(&core0R::feedbackCallbackNTh, this, _1, _2);
      send_goal_options.result_callback = std::bind(&core0R::resultCallbackNTh, this, _1);
      NavThPoseAC->async_send_goal(goal_msg, send_goal_options);
    }



    // for (int i = 1;i < pose.size();i+=1)
    // {
    //   goal_msg.poses.push_back(sendNav2Goal(stof(pose[i][0]),stof(pose[i][1]), stof(pose[i][2]), stof(pose[i][3]), stof(pose[i][4]), stof(pose[i][5]), stof(pose[i][6])));
    // }
    // auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    // send_goal_options.feedback_callback = std::bind(&core0R::feedbackCallbackNTh, this, _1, _2);
    // send_goal_options.result_callback = std::bind(&core0R::resultCallbackNTh, this, _1);
    // NavThPoseAC->async_send_goal(goal_msg, send_goal_options);
}

void core0R::core0_repeat()
{
    vector<vector<string>> pose = read("core0_teach.csv");
    for (long unsigned int i = 1;i < pose.size();i+=1)
    {
        acummulated_poses_.push_back(sendNav2Goal(stof(pose[i][0]),stof(pose[i][1]), stof(pose[i][2]), stof(pose[i][3]), stof(pose[i][4]), stof(pose[i][5]), stof(pose[i][6])));
    }
    rclcpp::sleep_for(std::chrono::milliseconds(time0));
    marker_update();
    // rclcpp::sleep_for(std::chrono::milliseconds(time0));
    // sendGoal();
}

void core0R::feedbackCallbackNTh(GoalHandleNavigateThroughPoses::SharedPtr,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
    RCLCPP_INFO(get_logger(), "number_of_poses_remaining = %d", feedback->number_of_poses_remaining);
    RCLCPP_INFO(get_logger(), "number_of_recoveries = %d", feedback->number_of_recoveries);
    
  }

void core0R::resultCallbackNTh(const GoalHandleNavigateThroughPoses::WrappedResult & result)
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

void core0R::updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_)
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
}

void core0R::resetUniqueId()
{
  unique_id = 0;
}

int core0R::getUniqueId()
{
  int temp_id = unique_id;
  unique_id += 1;
  return temp_id;
}

geometry_msgs::msg::PoseStamped core0R::startNav2sendGoal(double x,double y,double z,double ox,double oy,double oz,double ow)
{
    geometry_msgs::msg::PoseStamped start_goal_msg;
    start_goal_msg.header.stamp = this->now();
    start_goal_msg.header.frame_id = "map";
    start_goal_msg.pose.position.x = x;
    start_goal_msg.pose.position.y = y;
    start_goal_msg.pose.position.z = z;
    start_goal_msg.pose.orientation.x = ox;
    start_goal_msg.pose.orientation.y = oy;
    start_goal_msg.pose.orientation.z = oz;
    start_goal_msg.pose.orientation.w = ow;
    Nav2GoalPub->publish(start_goal_msg);
    return start_goal_msg;
}

geometry_msgs::msg::PoseStamped core0R::sendNav2Goal(double x,double y,double z,double ox,double oy,double oz,double ow)
{
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
    return goal_msg;
}

vector<vector<string>> core0R::read(std::string read_csv)
{
    ifstream file;
    vector<vector<string>> content;
    vector<string> row;
    string line, word;
            // file.open("/home/user/sar/docker/arm64/packages/ros2_src/core0_teach.csv");
            file.open(read_csv);
            file>>line;
            string str = line;
            stringstream ss(str);
            vector<string> v;
            if(file.is_open())
            {
            while(getline(file, line))
            {
                row.clear();
            stringstream str(line);
            
            while(getline(str, word, ','))
            row.push_back(word);
            content.push_back(row);
            }
            }
            else
            {
                cout<<"Could not open the file\n";
            }
    return content;

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<core0R>();
  node->core0_repeat();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

