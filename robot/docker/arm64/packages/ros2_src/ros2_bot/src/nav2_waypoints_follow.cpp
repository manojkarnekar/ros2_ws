#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
// FollowWaypoints
#include "rclcpp/time.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;


class Nav2Client : public rclcpp::Node
{
public:
  void sendGoal()
  {
    nav2_pose(1.0, 0.0, 0.0, 1.0);
    // nav2_pose(0.0, 1.0, 0.0, 1.0);
    // nav2_pose(1.0, 1.0, 0.0, 1.0);
    std::cout << "Start waypoint" << std::endl;
    startWaypointFollowing(acummulated_poses_);
    acummulated_poses_.clear();
  }
  explicit Nav2Client(): Node("nav2_waypoints_goal")
  {
    waypoint_follower_action_client_ =
    rclcpp_action::create_client<nav2_msgs::action::FollowWaypoints>(
    client_node_,
    "follow_waypoints");
    auto options = rclcpp::NodeOptions().arguments(
    {"--ros-args --remap __node:=navigation_dialog_action_client"});
    client_node_ = std::make_shared<rclcpp::Node>("_", options);
    
  }
private:
  using WaypointFollowerGoalHandle = rclcpp_action::ClientGoalHandle<nav2_msgs::action::FollowWaypoints>;
  rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SharedPtr waypoint_follower_action_client_;
  nav2_msgs::action::FollowWaypoints::Goal waypoint_follower_goal_;

  WaypointFollowerGoalHandle::SharedPtr waypoint_follower_goal_handle_;
  std::chrono::milliseconds server_timeout_;
  std::vector<geometry_msgs::msg::PoseStamped> acummulated_poses_;
  void startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses);
  void nav2_pose(float t_x, float t_y, float o_z, float o_w);
  rclcpp::Node::SharedPtr client_node_;

};

void Nav2Client::startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(get_logger(), "follow_waypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;

  RCLCPP_DEBUG(get_logger(), "Sending a path of %zu waypoints:",
    waypoint_follower_goal_.poses.size());
  for (auto waypoint : waypoint_follower_goal_.poses) {
    RCLCPP_DEBUG(get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::FollowWaypoints>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      waypoint_follower_goal_handle_.reset();
    };

  auto future_goal_handle =
    waypoint_follower_action_client_->async_send_goal(waypoint_follower_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  waypoint_follower_goal_handle_ = future_goal_handle.get();
  if (!waypoint_follower_goal_handle_) {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    return;
  }

  // timer_.start(200, this);
}

void Nav2Client::nav2_pose(float t_x, float t_y, float o_z, float o_w)
{   

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = t_x;
    goal_pose.pose.position.y = t_y;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = o_z;
    goal_pose.pose.orientation.w = o_w;
    acummulated_poses_.push_back(goal_pose);

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  node->sendGoal();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}