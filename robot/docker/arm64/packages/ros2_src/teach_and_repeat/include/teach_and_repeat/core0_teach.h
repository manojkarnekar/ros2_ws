#ifndef core0_teach_H
#define core0_teach_H

#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/int16.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;

class core0T : public rclcpp::Node
{
  public:
    core0T() 
    : Node("core0T_node")
    {
        this->declare_parameter<std::string>("parent_frame", "map");
        this->get_parameter("parent_frame", parent_frame_);
        this->declare_parameter<std::string>("child_frame", "base_link");
        this->get_parameter("child_frame", child_frame_);

        timer_ = this->create_wall_timer(500ms, std::bind(&core0T::tf_update, this)); 
        lmotor_cmd_sub = this->create_subscription<std_msgs::msg::Int16>("/lmotor_cmd", 10, std::bind(&core0T::lmotor_cmd_cb, this, _1));
        rmotor_cmd_sub = this->create_subscription<std_msgs::msg::Int16>("/rmotor_cmd", 10, std::bind(&core0T::rmotor_cmd_cb, this, _1));
        pose_sub = this->create_subscription<std_msgs::msg::Int16>("/insert_pose", 10, std::bind(&core0T::insert_pose_cb, this, _1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    }
  public:
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr lmotor_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr rmotor_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr pose_sub;
    void lmotor_cmd_cb(const std_msgs::msg::Int16::SharedPtr msglm);
    void rmotor_cmd_cb(const std_msgs::msg::Int16::SharedPtr msgrm);
    void insert_pose_cb(const std_msgs::msg::Int16::SharedPtr msgip);
    void tf_update();
    void write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset);
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::string parent_frame_;
    std::string child_frame_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped transformStamped;
    vector<float> px;
    vector<float> py;
    vector<float> pz;
    vector<float> ox;
    vector<float> oy;
    vector<float> oz;
    vector<float> ow;
    vector<float> PoseIndex;
    float p_x,p_y,p_z,o_x,o_y,o_z,o_w;
    int lmData,rmData,ipData;
};

#endif
