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

void core0T::lmotor_cmd_cb(const std_msgs::msg::Int16::SharedPtr msglm)   
{ 
   lmData = msglm->data;

}
void core0T::rmotor_cmd_cb(const std_msgs::msg::Int16::SharedPtr msgrm)   
{ 
   rmData = msgrm->data;

}
void core0T::insert_pose_cb(const std_msgs::msg::Int16::SharedPtr msgip)
{
  ipData = msgip->data;
}

void core0T::tf_update()
{
try {
            transformStamped = tf_buffer_->lookupTransform(
            parent_frame_.c_str(), child_frame_.c_str(),
            tf2::TimePointZero);
            p_x = transformStamped.transform.translation.x;
            p_y = transformStamped.transform.translation.y;
            p_z = transformStamped.transform.translation.z;
            o_x = transformStamped.transform.rotation.x;
            o_y = transformStamped.transform.rotation.y;
            o_z = transformStamped.transform.rotation.z;
            o_w = transformStamped.transform.rotation.w;
        } 
    catch (tf2::TransformException & ex) 
    {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            parent_frame_.c_str(), child_frame_.c_str(), ex.what());
    }
    

  if (lmData > 0 || rmData > 0)
  {
    px.push_back(p_x);
    py.push_back(p_y);
    pz.push_back(p_z);
    ox.push_back(o_x);
    oy.push_back(o_y);
    oz.push_back(o_z);
    ow.push_back(o_w);
    // Wrap into a vector
    std::vector<std::pair<std::string, std::vector<float>>> vals = {{"px", px}, {"py", py}, {"pz", pz}, {"ox",ox}, {"oy",oy}, {"oz",oz}, {"ow",ow}};
    // Write the vector to CSV
    write_csv("core0_teach.csv", vals);
  }
  if (ipData == 1)
  {
    int poseVecSize =px.size();
    PoseIndex.push_back(poseVecSize);
    cout<<"poseVecSize == "<<poseVecSize<<endl;
  }
  std::vector<std::pair<std::string, std::vector<float>>> posevals = {{"PoseIndex", PoseIndex}};
  write_csv("core0_flash_pose.csv", posevals);
    
}

void core0T::write_csv(std::string filename, std::vector<std::pair<std::string, std::vector<float>>> dataset)
{
    // Create an output filestream object
    std::ofstream myFile(filename);

     // Send column names to the stream
    for(int j = 0; j < dataset.size(); ++j)
    {
        myFile << dataset.at(j).first;
        if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
    }
    myFile << "\n";
    
    // Send data to the stream
    for(int i = 0; i < dataset.at(0).second.size(); ++i)
    {
        for(int j = 0; j < dataset.size(); ++j)
        {
            myFile << dataset.at(j).second.at(i);
            if(j != dataset.size() - 1) myFile << ","; // No comma at end of line
        }
        myFile << "\n";
    }
    
    // Close the file
    myFile.close();
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<core0T>(); 
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


