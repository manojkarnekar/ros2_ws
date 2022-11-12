#ifndef afs_odom_H
#define afs_odom_H

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Afs_Odom : public rclcpp::Node
{
  public:
    Afs_Odom()
    : Node("Afs_Odom_node") 
    {
        drive_encoder = this->create_subscription<std_msgs::msg::Int32>("/drive_encoder", 10, std::bind(&Afs_Odom::drive_encoder_Cb, this, _1));
        heading_fb = this->create_subscription<std_msgs::msg::Float32>("/heading_fb", 10, std::bind(&Afs_Odom::heading_fb_Cb, this, _1));
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/afs_odom", 10);
        alpha_odom_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/alphasense/odom", 10);
        odom_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_target_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // timer_ = this->create_wall_timer(20ms, std::bind(&Afs_Odom::update, this));
        
    }
  public:
    void heading_fb_Cb(const std_msgs::msg::Float32::SharedPtr msg);
    void drive_encoder_Cb(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_encoder;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_fb;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr alpha_odom_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void init_variables();
    void update();

    double encoder_min_val =  -2147483647;
    double encoder_max_val =  2147483647; // data type Int32 so 2pow(32) = 4294967296   (4294967296/2=2147483647)  
    double ticks_meter = 23468;
    double base_width = 0.3646;
    double base_length = 0.892;
    double encoder_low_wrap = ((encoder_max_val - encoder_min_val) * 0.3) + encoder_min_val ;
    double encoder_high_wrap = ((encoder_max_val - encoder_min_val) * 0.7) + encoder_min_val ;
    double rate = 50.0;
    double motor_direction = 1;
    double encoder_direction = 1;
    double angle = 0;
    double angle_prev = 0;
    double dr_enc_prev = 0;
    double drive_enc = 0; 
    double drive_dist = 0;
    double dr_mult = 0;
    double X = 0;
    double Y = 0;
    double Vx = 0;
    double Vy = 0;
    double Wz = 0;
    double Vd = 0;
    double heading = 0;
    double dr_enc = 0;
    double dt = 0;
    double angle_mean = 0;
    double Z = 0.0;
    bool odom_tf_pub = false;
    double drive_enc_prev = 0;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_target_tf;
    void send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z);
};

#endif