#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/time.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr multi_goal_client_ptr_;

  explicit Nav2Client(): Node("nav2_send_multiple_goal")
  {
    this->multi_goal_client_ptr_  = rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");
    
  }

  void sendGoal(void) {
    while (!this->multi_goal_client_ptr_->wait_for_action_server()) {
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    auto goal_msg = NavigateThroughPoses::Goal();

    // auto pose_ = nav2_pose(-2.0, 0.0, 0.0, 1.0);
    goal_msg.poses.push_back(nav2_pose(1.0, 0.0, 0.0, 1.0));

    // auto pose_ = nav2_pose(1.0, 0.0, 0.0, 1.0);
    goal_msg.poses.push_back(nav2_pose(2.0, 0.0, 0.0, 1.0));
    // goal_msg.poses.push_back(nav2_pose(0.0, 2.0, 0.0, 1.0));

    // auto pose_ = nav2_pose(2.0, 0.0, 0.0, 1.0);
    // goal_msg.poses.push_back(pose_);
    

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.feedback_callback = std::bind(&Nav2Client::feedbackCallback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&Nav2Client::resultCallback, this, _1);

    multi_goal_client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }
  //feedback
  void feedbackCallback(GoalHandleNavigateThroughPoses::SharedPtr,const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f", feedback->distance_remaining);
  }
  //result
  void resultCallback(const GoalHandleNavigateThroughPoses::WrappedResult & result)
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
public:
  geometry_msgs::msg::PoseStamped nav2_pose(float t_x, float t_y, float o_z, float o_w);
  // vector<float> v {};
  
};

geometry_msgs::msg::PoseStamped Nav2Client::nav2_pose(float t_x, float t_y, float o_z, float o_w)
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

    return goal_pose;

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