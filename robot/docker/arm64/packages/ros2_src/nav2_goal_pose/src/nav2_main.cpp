#include "nav2_goal_pose/nav2_goal_pose.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Utility>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}