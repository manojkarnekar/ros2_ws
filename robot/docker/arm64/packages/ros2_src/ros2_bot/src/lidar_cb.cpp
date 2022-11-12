#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#define RAD2DEG(x) ((x)*180./M_PI)

#include "Lidar.hpp"

Lidar lidar;

using std::placeholders::_1;
using namespace std::chrono_literals;

class Lidar_sp : public rclcpp::Node
{
  public:
    Lidar_sp()
    : Node("Lidar_sp_odom")
    {
      // topic_name_ = this->declare_parameter("topic_name", std::string("scan"));
      lidar_info_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&Lidar_sp::scanCb, this, _1));
      lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_custom", rclcpp::QoS(rclcpp::KeepLast(10)));
      int cnt=0;

      
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan);
    rclcpp::TimerBase::SharedPtr timer_;
    int cnt=0;
	

};

void Lidar_sp::scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    cnt=0;
    lidar.frame_id_ = scan->header.frame_id;
    lidar.angle_min_ = scan->angle_min;
    lidar.angle_max_ = scan->angle_max;
    lidar.angle_increment_ = scan->angle_increment;
    lidar.time_increment_ = scan->time_increment;
    lidar.scan_time_ = scan->scan_time;
    lidar.range_min_ = scan->range_min;
    lidar.range_max_ = scan->range_max;

    for(auto range:scan->ranges)
    {
        lidar.ranges_.push_back(range);
    }

    for(auto intensity:scan->intensities)
    {
        lidar.intensities_.push_back(intensity);
    }

    

    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = lidar.frame_id_;
    scan_msg.angle_min = lidar.angle_min_;
    scan_msg.angle_max = lidar.angle_max_;
    scan_msg.angle_increment = lidar.angle_increment_;
    scan_msg.time_increment = lidar.time_increment_;
    scan_msg.scan_time = lidar.scan_time_;
    scan_msg.range_min = lidar.range_min_;
    scan_msg.range_max = lidar.range_max_;

    scan_msg.ranges.resize(lidar.ranges_.size());
    scan_msg.intensities.resize(lidar.intensities_.size());

    // Front Left
    // for(int i=0; i<480;i++)
    // {
    //     scan_msg.ranges[i] = lidar.ranges_[i];
    //     scan_msg.intensities[i] = lidar.intensities_[i];
    // }
    

    // Front Right 
    // for(int i=2835; i<3240;i++)
    // {
    //     scan_msg.ranges[i] = lidar.ranges_[i];
    //     scan_msg.intensities[i] = lidar.intensities_[i];
    // }
    
    // Back Left
    // for(int i=1215; i<1620;i++)
    // {
    //     scan_msg.ranges[i] = lidar.ranges_[i];
    //     scan_msg.intensities[i] = lidar.intensities_[i];
    // }

    // Back Right
    // for(int i=1620; i<2025;i++)
    // {
    //     scan_msg.ranges[i] = lidar.ranges_[i];
    //     scan_msg.intensities[i] = lidar.intensities_[i];
    // }

    // Right
    // for(int i=2025; i<2835;i++)
    // {
    //     scan_msg.ranges[i] = lidar.ranges_[i];
    //     scan_msg.intensities[i] = lidar.intensities_[i];
    // }

    // Left
    for(int i=405; i<1215;i++)
    {
        scan_msg.ranges[i] = lidar.ranges_[i];
        scan_msg.intensities[i] = lidar.intensities_[i];
    }
    
    

    lidar_pub->publish(scan_msg);
    lidar.intensities_.clear();
    lidar.ranges_.clear();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar_sp>());
  rclcpp::shutdown();
  return 0;
}