#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <memory>
#include <string>


using std::placeholders::_1;
using namespace std::chrono_literals;

class TwistMotorControl : public rclcpp::Node
{
  public:
    TwistMotorControl()
    : Node("TwistMotorControl")
    {
      cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&TwistMotorControl::cmd_vel_cb, this, _1));
      
      pub_lmotor = this->create_publisher<std_msgs::msg::Int16>("lmotor_cmd", 10);
      pub_rmotor = this->create_publisher<std_msgs::msg::Int16>("rmotor_cmd", 10);

      timer_ = this->create_wall_timer(1ms, std::bind(&TwistMotorControl::spin, this));
      init_variables();
    }

  private:
    void cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg);
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;

    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_lmotor;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_rmotor;
    rclcpp::TimerBase::SharedPtr timer_;

    float left, right, ticks_since_target, dx,dy,dr;
	  double timeout_ticks, w, rate, K, a;

    void init_variables();
    void spinOnce();
    void spin();

};

void TwistMotorControl::init_variables()
{
  left = 0;
	right = 0;
	dx = dy = dr =0;
	w = 0.465;
	rate = 50;
	timeout_ticks = 20;
  K = 542;
  a = 0.075;
}

void TwistMotorControl::spin()
{
  rclcpp::Rate r(rate);

  if (ticks_since_target < timeout_ticks)
  {
    spinOnce();
  }
  
  ticks_since_target += 1;
  r.sleep();
}

void TwistMotorControl::spinOnce()
{
  left = K*(( 1.0 * dx ) - (dr * w /2));
	right = K*(( 1.0 * dx ) + (dr * w /2));
	// std::cout << "cmd_vel subscribing"<<" dx "<<dx<<" dy "<<dy<<" dr "<<dr<< std::endl;
  // std::cout << "l_&_rmotor cmd_vel subscribing"<<" K "<<K<<" w "<<w<<" left "<<left<<" right "<<right<< std::endl;
  
	std_msgs::msg::Int16 left_;
	std_msgs::msg::Int16 right_;

	left_.data = left;
	right_.data = right;

	pub_lmotor->publish(left_);
	pub_rmotor->publish(right_);

}

void TwistMotorControl::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    rclcpp::Time now = this->get_clock()->now();
    // std::cout << "----------------------------------------------------------------------------"<< std::endl;
    ticks_since_target = 0;
    dx = msg->linear.x;
    dy = msg->linear.y;
    dr = msg->angular.z;
    // std::cout << "cmd_vel subscribing" <<dx<<dy<<dr<< std::endl;

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistMotorControl>());
  rclcpp::shutdown();
  return 0;
}