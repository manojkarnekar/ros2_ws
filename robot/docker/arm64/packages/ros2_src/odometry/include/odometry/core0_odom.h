#ifndef core0_odom_H
#define core0_odom_H
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Core0_odom : public rclcpp::Node
{
  public:
    Core0_odom()
    : Node("Core0_odom_odom")
    {
      this->declare_parameter("odom_tf_pub", true);
      this->declare_parameter("ticks_meter", 664.0);
      this->declare_parameter("base_width", 0.470);
      this->declare_parameter("rate", 10.0);

      odom_tf_pub = get_parameter("odom_tf_pub").as_bool();
      ticks_meter = get_parameter("ticks_meter").as_double();
      base_width = get_parameter("base_width").as_double();
      rate = get_parameter("rate").as_double();

      // this->declare_parameter<std::string>("odom_tf_pub", "false");
      l_whell_encoder = this->create_subscription<std_msgs::msg::Int16>(
      "lwheel_encoder", 10, std::bind(&Core0_odom::leftencoderCb, this, _1));

      r_whell_encoder = this->create_subscription<std_msgs::msg::Int16>(
      "rwheel_encoder", 10, std::bind(&Core0_odom::rightencoderCb, this, _1));

      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
	    // timer_ = this->create_wall_timer(20ms, std::bind(&Core0_odom::update, this));
	    odom_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      static_target_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

  private:
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr l_whell_encoder;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr r_whell_encoder;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    void leftencoderCb(const std_msgs::msg::Int16::SharedPtr left_ticks);
    void rightencoderCb(const std_msgs::msg::Int16::SharedPtr right_ticks);
	  rclcpp::TimerBase::SharedPtr timer_;

	  std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_target_tf;
    void send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z);

    double encoder_min = -65535;
    double encoder_max = 65535;
    double encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
    double encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;
    double prev_lencoder = 0;
    double prev_rencoder = 0;
    double lmult = 0;
    double rmult = 0;
    double left = 0;
    double right = 0;
    double rate;
    double enc_left = 0;
    double enc_right = 0;
    double ticks_meter;  // 3185 //660
    double base_width;
    double dx = 0.0;
    double dr = 0.0;
	  double x_final = 0;
    double y_final = 0;
    double theta_final = 0;
    double elapsed = 0;
    double d_left = 0;
    double d_right = 0; 
    double d = 0;
    double th = 0;
    double x = 0;
    double y = 0;
	  double dt = 0;
    bool odom_tf_pub;
	  // rclcpp::Time t_next, then, current_time, last_time;
    rclcpp::Time t_next = this->get_clock()->now() + rclcpp::Duration(1/rate, 0);
    rclcpp::Time then = this->get_clock()->now();
    rclcpp::Time current_time = this->get_clock()->now();
    rclcpp::Time last_time = this->get_clock()->now();

	  
    
    // this->get_parameter("odom_tf_pub", odom_tf_pub);
    

	  void update();
	

};

#endif