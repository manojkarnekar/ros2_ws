#include "control/core0_control.h"

void Core0_control::spinOnce()
{
  left = K*(( 1.0 * dx ) - (dr * w /2));
  right = K*(( 1.0 * dx ) + (dr * w /2));

  std_msgs::msg::Int16 left_;
  std_msgs::msg::Int16 right_;

  left_.data = left;
  right_.data = right;
  pub_lmotor->publish(left_);
  pub_rmotor->publish(right_);

}

void Core0_control::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    dx = msg->linear.x;
    dy = msg->linear.y;
    dr = msg->angular.z;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Core0_control>());
  rclcpp::shutdown();
  return 0;
}