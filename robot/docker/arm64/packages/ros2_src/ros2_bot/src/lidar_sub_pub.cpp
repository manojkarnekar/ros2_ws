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
      lidar_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_f15", rclcpp::QoS(rclcpp::KeepLast(10)));
	    timer_ = this->create_wall_timer(71ms, std::bind(&Lidar_sp::update, this));
      int cnt=0;

      
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_info_sub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_pub;
    void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan);
    rclcpp::TimerBase::SharedPtr timer_;
    void update();
    int cnt=0;
	

};


void Lidar_sp::update()
{
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

    if (lidar.ranges_.size() !=0)
    {
      lidar.ranges_pop.clear();
      lidar.intensities_pop.clear();

      scan_msg.ranges.resize(lidar.ranges_.size());
      scan_msg.intensities.resize(lidar.intensities_.size());

      for(int i=0; i<lidar.ranges_.size();i++)
      {
        scan_msg.ranges[i] = lidar.ranges_[i];
        lidar.ranges_pop.push_back(lidar.ranges_[i]);
        
        
        // std::cout<<lidar.ranges_[i]<<" ";
      }

      for(int i=0; i<lidar.intensities_.size();i++)
      {
        scan_msg.intensities[i] = lidar.intensities_[i];
        lidar.intensities_pop.push_back(lidar.intensities_[i]);

        // std::cout<<lidar.intensities_[i]<<" ";
      }

      lidar_pub->publish(scan_msg);
      cout<<lidar.ranges_.size()<<std::endl;
      lidar.intensities_.clear();
      lidar.ranges_.clear();

      


      
    }
    else
    {
      scan_msg.ranges.resize(lidar.ranges_pop.size());
      scan_msg.intensities.resize(lidar.intensities_pop.size());

      // cout<<lidar.ranges_pop.size()<<" "<<lidar.intensities_pop.size()<<std::endl;

      for(int i=0; i<lidar.ranges_pop.size();i++)
      {
        scan_msg.ranges[i] = lidar.ranges_pop[i];
        // std::cout<<lidar.ranges_pop[i]<<" ";
      }

      for(int i=0; i<lidar.intensities_pop.size();i++)
      {
        scan_msg.intensities[i] = lidar.intensities_pop[i];
        // std::cout<<lidar.intensities_pop[i]<<" ";
      }

      lidar_pub->publish(scan_msg);
      
      // lidar.ranges_pop.clear();
      // lidar.intensities_pop.clear();
      
      // cout<<scan_msg.ranges.size()<<std::endl;
    }

    
    // cout<<scan_msg.ranges.size()<<" "<<scan_msg.intensities.size()<<std::endl;

}


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

    // update();

    

}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Lidar_sp>());
  rclcpp::shutdown();
  return 0;
}