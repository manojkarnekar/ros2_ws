#ifndef core0_control_H
#define core0_control_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Core0_control : public rclcpp::Node
{
  public:
    Core0_control()
    : Node("Core0_control")
    {
      cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Core0_control::cmd_vel_cb, this, _1));
      
      pub_lmotor = this->create_publisher<std_msgs::msg::Int16>("lmotor_cmd", 10);
      pub_rmotor = this->create_publisher<std_msgs::msg::Int16>("rmotor_cmd", 10);

      timer_ = this->create_wall_timer(50ms, std::bind(&Core0_control::spinOnce, this));

      this->declare_parameter("K", 270.56);
      this->declare_parameter("w", 0.472);

      K = get_parameter("K").as_double();
      w = get_parameter("w").as_double();
      
    }

  private:
    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_lmotor;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_rmotor;
    rclcpp::TimerBase::SharedPtr timer_;

    float left = 0;
    float right = 0;
	  float dx = 0;
    float dy = 0;
    float dr = 0;

    double w; // 0.465 // 0.472
    double rate = 50;
    double timeout_ticks = 20;
    double K; // 542.55 //270.56
    double a = 0.075;

    void spinOnce();
    void spin();

};




#endif