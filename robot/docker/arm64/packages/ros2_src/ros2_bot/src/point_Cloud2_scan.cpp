#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <memory>
#include <string>
#include <math.h>


using std::placeholders::_1;
using namespace std::chrono_literals;

class Pcl_to_scan : public rclcpp::Node
{
  public:
    Pcl_to_scan()
    : Node("Pcl_to_scan")
    {
      min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
      max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
      angle_min_ = this->declare_parameter("angle_min", -M_PI);
      angle_max_ = this->declare_parameter("angle_max", M_PI);
      angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
      scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
      range_min_ = this->declare_parameter("range_min", 0.0);
      range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
      inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
      use_inf_ = this->declare_parameter("use_inf", true);

      auto pc_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("intel_realsense_r200_depth/points", rclcpp::QoS(10), std::bind(&Pcl_to_scan::cloudCallback, this, std::placeholders::_1));
      scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("depth/scan", rclcpp::SensorDataQoS());
      
      // timer_ = this->create_wall_timer(20ms, std::bind(&Pcl_to_scan::spinOnce, this));
    }

  private:
    void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg);
    std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::LaserScan>> scan_pub;
    // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub;
    // rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    // sensor_msgs::msg::LaserScan scan_msg;
    // auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    int input_queue_size_;
    std::string target_frame_;
    double tolerance_, min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
    bool use_inf_;
    double inf_epsilon_;
};



void Pcl_to_scan::cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;
  scan_msg->angle_min = -M_PI;
  scan_msg->angle_max = M_PI;
  scan_msg->angle_increment = M_PI / 180.0;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = 1.0 / 30.0;
  scan_msg->range_min = 0.0;
  scan_msg->range_max = 3.0;

  uint32_t ranges_size = std::ceil(
    (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);
  
  scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());

  // Iterate through pointcloud
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"), iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
    iter_x != iter_x.end(); 
    ++iter_x, ++iter_y, ++iter_z)
    {
      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) 
      {
        RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
        continue;
      }

      if (*iter_z > max_height_ || *iter_z < min_height_) 
      {
        RCLCPP_DEBUG(
          this->get_logger(),
          "rejected for height %f not in range (%f, %f)\n",
          *iter_z, min_height_, max_height_);
        continue;
      }

      double range = hypot(*iter_x, *iter_y);

      if (range < range_min_) 
      {
        RCLCPP_DEBUG(
          this->get_logger(),
          "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
          range, range_min_, *iter_x, *iter_y, *iter_z);
        continue;
      }

      if (range > range_max_)
      {
        RCLCPP_DEBUG(
          this->get_logger(),
          "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
          range, range_max_, *iter_x, *iter_y, *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);

      if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) 
      {
        RCLCPP_DEBUG(
          this->get_logger(),
          "rejected for angle %f not in range (%f, %f)\n",
          angle, scan_msg->angle_min, scan_msg->angle_max);
        continue;
      }

      // overwrite range at laserscan ray if new range is smaller
      int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;

      if (range < scan_msg->ranges[index]) 
      {
        scan_msg->ranges[index] = range;
      }
    }
  scan_pub->publish(std::move(scan_msg));

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pcl_to_scan>());
  rclcpp::shutdown();
  return 0;
}
