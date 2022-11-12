#ifndef afs_control_H
#define afs_control_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;

class AfsControl : public rclcpp::Node
{
  public:
    AfsControl()
    : Node("AfsControl_node")
    {
        CmdVelSub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&AfsControl::CmdVelCb, this, _1));
        pub_steer = this->create_publisher<std_msgs::msg::Float32>("/heading", 50);
        pub_steer_vel = this->create_publisher<std_msgs::msg::UInt8>("/steer_speed", 50);
        pub_drive = this->create_publisher<std_msgs::msg::UInt16>("/throttle_state", 50);
        pub_drive_dir = this->create_publisher<std_msgs::msg::UInt8>("/drive_direction", 50);
       
        timer_ = this->create_wall_timer(20ms, std::bind(&AfsControl::update, this));   
    }
  public:
    void CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg);
    void update();

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr CmdVelSub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_steer;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_steer_vel;
    rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr pub_drive;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_drive_dir;
    rclcpp::TimerBase::SharedPtr timer_;
    // float steer = 0;
	float drive=0;
	float ticks_since_target=0;
	double timeout_ticks=20;
	double K=300;   
	double w=0.58;
	double bl = 0.892;
	double rate=50;
	double K_steer=1;
	double K_drive=728;
	double bw = 0.3646;
	float dx=0;
    float dy=0;
    float dr=0;
	float r = 0.2;
	float r_fact = 1.0/r;
    double steer_angle = 0;
	double drive_rn = 0;
	double steer_rn = 0;
	double l_drive = 0;
	double r_drive = 0;
};




#endif