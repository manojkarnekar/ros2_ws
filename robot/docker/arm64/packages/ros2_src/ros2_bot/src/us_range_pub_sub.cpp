#include <chrono>
#include <memory>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
// http://docs.ros.org/en/melodic/api/sensor_msgs/html/index-msg.html
//#include "sensor_msgs/Range.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"




using std::placeholders::_1;
using namespace std::chrono_literals;



class US_range_pub_sub : public rclcpp::Node
{
  public:
    US_range_pub_sub()
    : Node("US_range_pub_sub")
    {
      US_range_sub = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
      "/us_scan", 10, std::bind(&US_range_pub_sub::US_range_callback, this, _1));
      
 // MultiArrayLayout  layout        # specification of data layout
 // uint8[]           data          # array of data
//std_msgs/UInt8MultiArray.msg

// /range is the topic name
      //US_range_pub = this->create_publisher<sensor_msgs::msg::US_range>("US/range", 6);
      
// std::bind(&US_range_pub_sub # node name ::US_range_callback # create callback name, this, _1));
      
      US_range_pub_1 = this->create_publisher<sensor_msgs::msg::Range>("US1", 10);
      US_range_pub_2 = this->create_publisher<sensor_msgs::msg::Range>("US2", 10);
      US_range_pub_3 = this->create_publisher<sensor_msgs::msg::Range>("US3", 10);
      US_range_pub_4 = this->create_publisher<sensor_msgs::msg::Range>("US4", 10);
      US_range_pub_5 = this->create_publisher<sensor_msgs::msg::Range>("US5", 10);
      US_range_pub_6 = this->create_publisher<sensor_msgs::msg::Range>("US6", 10);
      
   
   // http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Range.html    
 //    sensor_msgs/Range.msg
//Raw Message Definition
//uint8 data
// Header header 
//uint8 ULTRASOUND=0   uint8 INFRARED=1    uint8 radiation_type   
// float32 field_of_view  float32 min_range       # minimum range value [m]
// float32 max_range       # maximum range value [m]
// float32 range           # range data [m] # (Note: values < range_min or > range_max

//uint8 ULTRASOUND=0 ,uint8 INFRARED=1, std_msgs/Header header, uint8 radiation_type, float32 field_of_view, float32 min_range, float32 max_range, float32 range

      
    }
    
private:
    void US_range_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr US_range_sub;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr US_range_pub_1;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr US_range_pub_2;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr US_range_pub_3;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr US_range_pub_4;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr US_range_pub_5;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr US_range_pub_6;
    // uint8_t d;
    // std::string frame_id;
    // rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr US_range_pub;

    void us_range(unsigned int d,std::string frame_id, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us);
    // void us_range(unsigned int d,std::string frame_id);
    // void us_range(uint8_t d,std::string frame_id);
    // void us_range(unsigned int d);



    
  //  https://docs.ros2.org/bouncy/api/rclcpp/classrclcpp_1_1_node.html
    
    float data = 0;
    int i = 0;
    uint8_t US1, US2, US3, US4, US5, US6;
    rclcpp::Time then = this->get_clock()->now();
    
    

    
    
};

void US_range_pub_sub::us_range(unsigned int d,std::string frame_id, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr us)
{
  sensor_msgs::msg::Range rangeMsg;
  double distance = d;
  // rclcpp::Time now = this->get_clock()->now();
  rangeMsg.header.stamp = this->now();
	rangeMsg.header.frame_id = frame_id;
	rangeMsg.radiation_type = 0,                     //0=ultrasonic, 1=IR
	rangeMsg.field_of_view = 0.05;
	rangeMsg.min_range = 0.0;
	rangeMsg.max_range = 214.0;
	rangeMsg.range = distance;
  us->publish(rangeMsg);
}





void US_range_pub_sub::US_range_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg)

    
{    

    unsigned int raw[6];
    
    for(int i=0; i<6; i++)
    {
      raw[i] = msg->data[i];
    }

    // double dustance = raw[0];
    // std::string frame_id = "sonar0";

    // sensor_msgs::msg::Range rangeMsg;
    // // rclcpp::Time now = this->get_clock()->now();
    // rangeMsg.header.stamp = this->now();
    // rangeMsg.header.frame_id = "sonar_fr1";
    // rangeMsg.radiation_type = 0,                     //0=ultrasonic, 1=IR
    // rangeMsg.field_of_view = 0.05;
    // rangeMsg.min_range = 0.0;
    // rangeMsg.max_range = 214.0;
    // rangeMsg.range = dustance;
    // US_range_pub_1->publish(rangeMsg);

    us_range(raw[0], "sonar0", US_range_pub_1);
    us_range(raw[1], "sonar1", US_range_pub_2);
    us_range(raw[2], "sonar2", US_range_pub_3);
    us_range(raw[3], "sonar3", US_range_pub_4);
    us_range(raw[4], "sonar4", US_range_pub_5);
    us_range(raw[5], "sonar5", US_range_pub_6);

    

		

    // std::cout << "US_data-->" <<raw[0]<<" "<<raw[1]<<" "<<raw[2]<<" "<<raw[3]<<" "<<raw[4]<<" "<<raw[5]<<" "<<std::endl;

    // US1 = msg->data.at(0);
    // US2 = msg->data.at(1);
    // US3 = msg->data.at(2);
    // US4 = msg->data.at(3);
    // US5 = msg->data.at(4);
    // US6 = msg->data.at(5);

    // US_range_pub_1->publish(raw[0]);
    // US_range_pub_2->publish(raw[1]);
    // US_range_pub_3->publish(raw[2]);
    // US_range_pub_4->publish(raw[3]);
    // US_range_pub_5->publish(raw[4]);
    // US_range_pub_6->publish(raw[5]);

}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<US_range_pub_sub>());
  rclcpp::shutdown();
  return 0;
}





