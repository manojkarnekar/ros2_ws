#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include<iostream>
#include <fstream>
#include <sstream>
#include <map>

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

std::shared_ptr<rclcpp::Node> node = nullptr;

void TopicCallback(const std_msgs::msg::String::SharedPtr msg)
{
    RCLCPP_INFO(node->get_logger(), "I heard : '%s'", msg->data.c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<rclcpp::Node>("nav2_goal_pose");
    auto sub = node->create_subscription<std_msgs::msg::String>("/my_topic", 10, TopicCallback);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}