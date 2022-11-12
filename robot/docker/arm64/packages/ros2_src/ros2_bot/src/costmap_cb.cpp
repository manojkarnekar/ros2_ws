#include <memory>
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class Nav_algo : public rclcpp::Node
{
  public:
    Nav_algo()
    : Node("Nav_algo")
    {
      costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/local_costmap/costmap", 10, std::bind(&Nav_algo::costmap_cb, this, _1));

    }

  private:
    void costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub;

};

void Nav_algo::costmap_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // RCLCPP_INFO(get_logger(), "subscribe a maptopic");
    std_msgs::msg::Header header = msg->header;
    nav_msgs::msg::MapMetaData info = msg->info;
    // std::cout <<"width-->"<<info.width<<" "<<"height-->"<<info.height<<std::endl;

    for (unsigned int j=0;j<msg->info.height;j++)
    {
      for (unsigned int i=0;i<msg->info.width;i++)
      {
        int8_t v = msg->data[j*msg->info.width + i];
        std::cout <<"width-->"<<v<<std::endl;
      }
    }
                    
    

}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nav_algo>());
  rclcpp::shutdown();
  return 0;
}