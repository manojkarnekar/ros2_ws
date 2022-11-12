#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::placeholders;
using std::placeholders::_1;
using namespace std;
namespace enc = sensor_msgs::image_encodings;

class zed2_LScan : public rclcpp::Node
{
public:
    zed2_LScan() 
    : Node("zed2_LScan")
    {
        rclcpp::QoS depth_qos(10);
        depth_qos.keep_last(10);
        depth_qos.best_effort();
        depth_qos.durability_volatile();
        DepthSub = create_subscription<sensor_msgs::msg::Image>("/zed2/zed_node/depth/depth_registered", depth_qos,std::bind(&zed2_LScan::depthCallback, this, _1) );
        RGBSub = create_subscription<sensor_msgs::msg::Image>("/zed2/zed_node/rgb/image_rect_color", depth_qos,std::bind(&zed2_LScan::rgbCallback, this, _1) );
        zed2_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("/zed2_scan", rclcpp::QoS(rclcpp::KeepLast(10)));
    }

public:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr DepthSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr RGBSub;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_scan_pub;
    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);
    void zed2_LaserScan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_scan_pub,float min_ang,float max_ang,float ang_incr);
    vector<vector<float>> d_col;
    vector<float> LSD;
};

void zed2_LScan::rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  imageCb(msg);
}

void zed2_LScan::depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) 
{
        // Get a pointer to the depth values casting the data
        // pointer to floating point
        float* depths = (float*)(&msg->data[0]);
        for(unsigned int i = 1;i<=msg->width;i+=3)
        {
          vector<float> v1;
          for(unsigned int j = 1;j<=msg->height;j+=3)
          {
            unsigned int centerIdx = i + msg->width * j;
            v1.push_back(depths[centerIdx]);
          }
          d_col.push_back(v1);
        }

        for (long unsigned int i = d_col.size()-1;i>0; i=i-1)
        {
        
            LSD.push_back(*min_element(d_col[i].begin(), d_col[i].end()));
            // cout<<*min_element(d_col[i].begin(), d_col[i].end())<<" ";
            // RCLCPP_INFO(get_logger(), "distance : %g m", *min_element(d_col[i].begin(), d_col[i].end()));
        }
        // cout<<LSD.size()<<endl;
        zed2_LaserScan(this->get_clock()->now(),"camera_link",LSD,zed2_scan_pub,-0.9599,0.9599,0.00897);
        d_col.clear();
        LSD.clear();

        // Output the measure
        // RCLCPP_INFO(get_logger(), "Center distance : %g m", depths[centerIdx]);
}

void zed2_LScan::zed2_LaserScan(rclcpp::Time now, std::string frame_id,vector<float> ranges, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_scan_pub,float min_ang,float max_ang,float ang_incr)
{
  sensor_msgs::msg::LaserScan zed2_scan_msg;
  zed2_scan_msg.header.stamp = now;
  zed2_scan_msg.header.frame_id = frame_id;
  zed2_scan_msg.angle_min = min_ang;  
  zed2_scan_msg.angle_max =  max_ang;
  zed2_scan_msg.angle_increment = ang_incr;
  zed2_scan_msg.time_increment = 0;
  zed2_scan_msg.scan_time = 0.0;
  zed2_scan_msg.range_min = 0.005;
  zed2_scan_msg.range_max = 1.00;
  zed2_scan_msg.ranges.resize(ranges.size());
  zed2_scan_msg.intensities.resize(ranges.size());
  zed2_scan_msg.ranges=ranges;
  
  vector<float> inten;
  for(long unsigned int i=0; i<ranges.size();i++)
  {
    inten.push_back(47.00);
  }
  zed2_scan_msg.intensities=inten;

  zed2_scan_pub->publish(zed2_scan_msg);

  inten.clear();
  ranges.clear();
}

void zed2_LScan::imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
{

  cv_bridge::CvImageConstPtr cv_ptr;
  
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    RCLCPP_INFO(get_logger(),"cv_bridge exception: %s", e.what());
    return;
  }

  for(unsigned int i = 1;i<msg->width;i+=3)
  {
    for(unsigned int j = 1;j<msg->height;j+=3)
    {
    
    cv::circle(cv_ptr->image, cv::Point(i , j), 1, CV_RGB(255,0,0), -1);

    }
  }

  // cv::circle(cv_ptr->image, cv::Point(msg->width/2 , msg->height/2), 10, CV_RGB(255,0,0));


  // cv::Mat mono8_img = cv::Mat(cv_ptr->image.size(), CV_8UC1);
  // cv::convertScaleAbs(cv_ptr->image, mono8_img, 100, 0.0);
  // cv::circle(cv_ptr->image, cv::Point(msg->width/2 , msg->height/2), 5, CV_RGB(255,0,0), -1);
  // cv::circle(cv_ptr->image, cv::Point(28 , 1), 5, CV_RGB(255,0,0), -1);

  cv::imshow("Image window", cv_ptr->image);
  cv::waitKey(1);
  
}



// The main function
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto depth_node = std::make_shared<zed2_LScan>();

    rclcpp::spin(depth_node);
    rclcpp::shutdown();
    return 0;
}
