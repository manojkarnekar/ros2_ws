#include "nav2_goal_pose/nav2_goal_pose.h"

void Utility::startNavigation(geometry_msgs::msg::PoseStamped pose)
{
  auto is_action_server_ready =
    navigation_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(),
      "navigate_to_pose action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal pose
  navigation_goal_.pose = pose;

  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateToPose will be called using the BT Navigator's default behavior tree.");

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      navigation_goal_handle_.reset();
    };

  auto future_goal_handle =
    navigation_action_client_->async_send_goal(navigation_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  navigation_goal_handle_ = future_goal_handle.get();
  if (!navigation_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }
}


void Utility::startWaypointFollowing(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    waypoint_follower_action_client_->wait_for_action_server(std::chrono::seconds(5));
  
  if (!is_action_server_ready) 
  {
    RCLCPP_ERROR(
      get_logger(), "follow_waypoints action server is not available."
      " Is the initial pose set?");
    return;
  }

  // Send the goal poses
  waypoint_follower_goal_.poses = poses;

  RCLCPP_DEBUG(
    get_logger(), "Sending a path of %zu waypoints:",
    waypoint_follower_goal_.poses.size());

  for (auto waypoint : waypoint_follower_goal_.poses) 
  {
    RCLCPP_DEBUG(
      get_logger(),
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
  if (!waypoint_follower_goal_handle_) 
  {
    RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
    return;
  }



}


void Utility::startNavThroughPoses(std::vector<geometry_msgs::msg::PoseStamped> poses)
{
  auto is_action_server_ready =
    nav_through_poses_action_client_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      client_node_->get_logger(), "navigate_through_poses action server is not available."
      " Is the initial pose set?");
    return;
  }

  nav_through_poses_goal_.poses = poses;
  RCLCPP_INFO(
    client_node_->get_logger(),
    "NavigateThroughPoses will be called using the BT Navigator's default behavior tree.");

  RCLCPP_DEBUG(
    client_node_->get_logger(), "Sending a path of %zu waypoints:",
    nav_through_poses_goal_.poses.size());
  for (auto waypoint : nav_through_poses_goal_.poses) {
    RCLCPP_DEBUG(
      client_node_->get_logger(),
      "\t(%lf, %lf)", waypoint.pose.position.x, waypoint.pose.position.y);
  }

  // Enable result awareness by providing an empty lambda function
  auto send_goal_options =
    rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();
  send_goal_options.result_callback = [this](auto) {
      nav_through_poses_goal_handle_.reset();
    };

  auto future_goal_handle =
    nav_through_poses_action_client_->async_send_goal(nav_through_poses_goal_, send_goal_options);
  if (rclcpp::spin_until_future_complete(client_node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(client_node_->get_logger(), "Send goal call failed");
    return;
  }

  // Get the goal handle and save so that we can check on completion in the timer callback
  nav_through_poses_goal_handle_ = future_goal_handle.get();
  if (!nav_through_poses_goal_handle_) {
    RCLCPP_ERROR(client_node_->get_logger(), "Goal was rejected by server");
    return;
  }
}
