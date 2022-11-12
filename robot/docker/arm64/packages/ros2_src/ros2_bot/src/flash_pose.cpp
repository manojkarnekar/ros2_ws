#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int16.hpp"
#include <rclcpp/time.hpp>

using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;

class core0FP : public rclcpp::Node
{
  public:
    core0FP() 
    : Node("core0FP_node")
    {
        FP_pub = this->create_publisher<std_msgs::msg::Int16>("/insert_pose", rclcpp::QoS(rclcpp::KeepLast(10)));
    }
   public:
     rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr FP_pub;
     void flash_pose();
     

};

void core0FP::flash_pose()
{
  int time0 = 100; 
  int time1 = 499;
  int insert_data = 1 ;
  int inactive_data=0;
  std_msgs::msg::Int16 msg;
  msg.data=insert_data;
  cout<<"insert_data == "<<insert_data<<endl;
  rclcpp::sleep_for(std::chrono::milliseconds(time0));
  FP_pub->publish(msg);
  rclcpp::sleep_for(std::chrono::milliseconds(time1));
  msg.data=inactive_data;
  FP_pub->publish(msg);
  rclcpp::shutdown();
  
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<core0FP>();
  node->flash_pose(); 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}