 #include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/int16.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace std;

class collision_M : public rclcpp::Node
{
  public:
    collision_M()
    : Node("collision_M")
    {
        lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&collision_M::lidar_cb, this, _1));
        CmdVelPub = this->create_publisher<geometry_msgs::msg::Twist>("/afs_cmd_vel" ,rclcpp::QoS(rclcpp::KeepLast(10)));
        cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SensorDataQoS(), std::bind(&collision_M::cmd_vel_cb, this, _1));
        mode_sub = this->create_subscription<std_msgs::msg::Int16>("/mode", 10, std::bind(&collision_M::mode_cb, this, _1));
    }
    public:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr CmdVelPub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr mode_sub;
    void lidar_cb(sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void vel_pub();
    void update();
    void cmd_vel_cb(geometry_msgs::msg::Twist::SharedPtr cmdvelmsg);
    void mode_cb(const std_msgs::msg::Int16::SharedPtr msgmode);
    vector<float> ScanData;
    float min_val,vx,oz;
    int mode;
};

void collision_M::cmd_vel_cb(geometry_msgs::msg::Twist::SharedPtr cmdvelmsg)
{
  vx=cmdvelmsg->linear.x;
  oz=cmdvelmsg->angular.z;
  if(0.4<min_val && min_val<0.6 )   //&& mode ==2
      {
        vx=0.0;
        oz=0.0;
        vel_pub();
      }
      else
      {
        vel_pub();
      }
}

void collision_M::vel_pub()
{
  geometry_msgs::msg::Twist vel_msg;
  vel_msg.linear.x=vx;
  vel_msg.linear.y=0;
  vel_msg.linear.z=0;
  vel_msg.angular.x=0;
  vel_msg.angular.y=0;
  vel_msg.angular.z=oz;
  CmdVelPub->publish(vel_msg);
}

void collision_M::lidar_cb(sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    scan_msg->header.stamp;
    float min_ang =scan_msg->angle_min;
    float max_ang=scan_msg->angle_max;
    float ang_incr=scan_msg->angle_increment;
    int NOD=(max_ang-min_ang)/ang_incr;
    int r_max = (-1.57-min_ang)/ang_incr;
    int l_min = (1.57-min_ang)/ang_incr;
    for (int i=0;i<r_max;i++)
    {
      if((scan_msg->ranges[i])>0.4)
      {
        ScanData.push_back(scan_msg->ranges[i]);
      }
    }
    for (int i=l_min;i<NOD;i++)
    {
      if((scan_msg->ranges[i])>0.4)
      {
        ScanData.push_back(scan_msg->ranges[i]);
      }
    }
    min_val=*min_element (ScanData.begin(), ScanData.end());

      ScanData.clear();
}

void collision_M::mode_cb(const std_msgs::msg::Int16::SharedPtr msgmode)  
{ 
   mode = msgmode->data;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<collision_M>(); 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}