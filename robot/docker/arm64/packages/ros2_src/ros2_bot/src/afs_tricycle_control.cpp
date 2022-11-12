#include <iostream>
#include <vector>
#include <cmath>
#include <bits/stdc++.h>
#include <math.h>
// #include "ros/ros.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define RAD2DEG(x) ((x)*180.0/M_PI)

using namespace std;

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}


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
    float steer(float steering_angle, float MAX_STEERING_ANGLE);
    float map(float x, float in_min, float in_max, float out_min, float out_max);
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
    float servo_steering_angle = 0;
};

float AfsControl::map(float x, float in_min, float in_max, float out_min, float out_max)
{
    float u = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    return u ;
}

float AfsControl::steer(float steering_angle, float MAX_STEERING_ANGLE)
{
    //steering function for ACKERMANN base
    
    steering_angle = constrain(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
    servo_steering_angle = map(steering_angle, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE, M_PI, 0) * (180 / M_PI);
    return steering_angle;
}

void AfsControl::update()
{
    // float steer_angle = steer(dr, 3.14);
    steer_angle = K_steer * (atan2(bl*dr, dx));
    drive_rn = K_drive * (hypot(bl*dr, dx));
    // int omega = (dx * tan(steer_angle))/bl;

    if(steer_angle>1.45)
    {
        steer_angle = 1.45;
    }

    if(steer_angle<-1.45)
    {
        steer_angle = -1.45;
    }

    // omwga = (linear_vel * tan(steer_angle))/ (wheel_to_base_fp_dis);

    std_msgs::msg::Float32 steer_;
    std_msgs::msg::UInt8 steer_vel_;
	std_msgs::msg::UInt16 drive_;
    std_msgs::msg::UInt8 drive_dir;

    // RCLCPP_INFO(get_logger(),"steer_angle->{%.2lf}, drive_rn->{%.2lf}",
    //             steer_angle,
    //             drive_rn);

    steer_.data = (steer_angle);
    // steer_vel_.data = omega;
    steer_vel_.data = 70;
	drive_.data = drive_rn;
    drive_dir.data = 0;

    pub_steer->publish(steer_);
    pub_steer_vel->publish(steer_vel_);
    pub_drive->publish(drive_);
    pub_drive_dir->publish(drive_dir);

    // ros::spinOnce();
}

void AfsControl::CmdVelCb(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    ticks_since_target = 0;
	dx = msg->linear.x;
	dy = msg->linear.y;
	dr = msg->angular.z;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AfsControl>());
  rclcpp::shutdown();
  return 0;
}



