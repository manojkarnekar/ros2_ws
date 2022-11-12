#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <map>
#include <bits/stdc++.h>
#include <math.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav2_util/geometry_utils.hpp"
// #include "zed2_full_bed_scan.hpp"


using namespace std;


using std::placeholders::_1;
using namespace std::chrono_literals;

using nav2_util::geometry_utils::orientationAroundZAxis;

class zed2_realignment_gp : public rclcpp::Node
{
  public:
    zed2_realignment_gp()
    : Node("zed2_realignment_gp")
    {
        this->declare_parameter<std::string>("parent_frame", "map");
        this->get_parameter("parent_frame", parent_frame_);
        this->declare_parameter<std::string>("child_frame", "camera_link");
        this->get_parameter("child_frame", child_frame_);
        timer_ = this->create_wall_timer(1ms, std::bind(&zed2_realignment_gp::tf_update, this));
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        zed2_realignment_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>("/cv_pose", rclcpp::SensorDataQoS(), std::bind(&zed2_realignment_gp::zed2_realignment_callback, this, _1)); 
    //   timer_ = this->create_wall_timer(71.42ms, std::bind(&zed2_realignment_gp::update, this));
        zed2_realignment_gp_pub = this->create_publisher<sensor_msgs::msg::LaserScan>("zed2_realignment_gp", rclcpp::QoS(rclcpp::KeepLast(10)));
        
        tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        frame_tf_pub_timer_ = this->create_wall_timer(100ms, std::bind(&zed2_realignment_gp::broadcast_timer_callback, this));
        this->broadcast_timer_callback();
        
        // pose_stamped_pub_L = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_l", 10);
        // pose_stamped_pub_M = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_m", 10);
        // pose_stamped_pub_R = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_r", 10);
        pose_stamped_pub_MF = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_mf", 10);
        wp_navigation_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", rclcpp::QoS(1).transient_local());


    }

  private:
    void zed2_realignment_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void update();
    void broadcast_timer_callback();
    // void nav2_pose(float x, float y, float O_z, float O_w, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub);
    void tf_update();
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr zed2_realignment_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr frame_tf_pub_timer_ ;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr zed2_realignment_gp_pub;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
    std::string parent_frame_;
    std::string child_frame_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped transformStamped;
    float T_x, T_y,T_z,O_x,O_y, O_z, O_w, gpx, gpy, gpz, gpth, m1, m2, th_i_d_, th_i_d, i_d, i_p, p1, p2, d1, d2, d3, th1, th2, bdfcx, bdfcz, bdfcthy;
    vector<float> z2b;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_L; 
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_M;
    // rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_R;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_MF;

    geometry_msgs::msg::PoseStamped nav2_pose(float t_x, float t_y, double theta);
    void updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_);
    void resetUniqueId();
    int getUniqueId();
    int unique_id {0};
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wp_navigation_markers_pub_;
    
};

void zed2_realignment_gp::zed2_realignment_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)    
{
    for(long unsigned int i=0; i<msg->data.size(); i++)
    {
        z2b.push_back(msg->data[i]);
    }
    p1= sqrt((z2b.at(2)-z2b.at(0))*(z2b.at(2)-z2b.at(0))+(z2b.at(3)-z2b.at(1))*(z2b.at(3)-z2b.at(1)));
    p2= sqrt((z2b.at(5)-z2b.at(3))*(z2b.at(5)-z2b.at(3))+(z2b.at(4)-z2b.at(2))*(z2b.at(4)-z2b.at(2)));
    i_p=sqrt((z2b.at(2)-z2b.at(12))*(z2b.at(2)-z2b.at(12))+(z2b.at(3)-z2b.at(11))*(z2b.at(3)-z2b.at(11)));
    d1= sqrt((z2b.at(0))*(z2b.at(0))+(z2b.at(1))*(z2b.at(1)));
    d2= sqrt((z2b.at(2))*(z2b.at(2))+(z2b.at(3))*(z2b.at(3)));
    d3= sqrt((z2b.at(4))*(z2b.at(4))+(z2b.at(5))*(z2b.at(5)));
    // i_d=abs(z2b.at(9)*1000);
    i_d=sqrt((z2b.at(11))*(z2b.at(11))+(z2b.at(12))*(z2b.at(12)));
    // cout<<" i_d "<<z2b.at(9);
    // th_i_d=asin(i_d/d2);
    // th_i_d=(i_d*0.05729)*(0.01745);
    // cout<<" th_i_d "<<th_i_d*(57.295);
    // bdx=2.5;
    th1= acos(((d1*d1)+(d2*d2)-(p1*p1))/(2*d1*d2));
    th2= acos(((d2*d2)+(d3*d3)-(p2*p2))/(2*d2*d3));
    th_i_d=acos(((d2*d2)+(i_d*i_d)-(i_p*i_p))/(2*d2*i_d));
    

    if (z2b.at(9)>0)
    {
      
      th_i_d=th_i_d;
    }
    else
    {
      th_i_d=-th_i_d;
      
    }

      m1=(d3*cos(th_i_d+th2)-(d1*cos(th_i_d-th1)))/(d3*sin(th_i_d+th2)-(d1*sin(th_i_d-th1)));
      // m2=-1/m1;
      th_i_d_=atan2(-m1,1);
      gpx=d2*sin(th_i_d)-(2*cos(th_i_d_));
      gpz=d2*cos(th_i_d)-(2*sin(th_i_d_));
      bdfcx=d2*sin(th_i_d);
      bdfcz=d2*cos(th_i_d);
      bdfcthy=((3.14/2) - th_i_d_);
      gpx=gpx;
      gpy=-gpz;
      gpth= ((3.14/2) - th_i_d_);

    // nav2_pose(T_z+gpx, T_x+gpy, -O_y+bdfcthy);
    // updateWpNavigationMarkers();
    // mid_pose = nav2_pose(x_m-1.0, y_m, 0.0);
    
    // cout<<" d1 "<<d1;
    // cout<<" d2 "<<d2;
    // cout<<" d3 "<<d3;
    // cout<<" gpx "<<gpx;
    // cout<<" gpy "<<gpy;
    // cout<<" gpth "<<(gpth*57.295);
    // cout<<" th_i_d "<<(th_i_d*57.295);
    // cout<<" m1 "<<m1;
    // cout<<" th_i_d_ "<<(th_i_d_*57.295);

    
    update();
    z2b.clear();
    // return z2b;
}

void zed2_realignment_gp::broadcast_timer_callback()
  {
    rclcpp::Time now = this->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = "map";
    t.child_frame_id = "bed_frame";
    t.transform.translation.x = T_x+bdfcx;
    t.transform.translation.y = T_y;
    t.transform.translation.z = T_z+bdfcz;
    t.transform.rotation.x = (3.14/2);
    t.transform.rotation.y = O_z+bdfcthy;
    t.transform.rotation.z = (3.14/2);
    t.transform.rotation.w = 1.0;
    tf_publisher_->sendTransform(t);
  }

void zed2_realignment_gp::tf_update()
{
  // geometry_msgs::msg::TransformStamped transformStamped;
  // geometry_msgs::msg::Twist msg;
  // msg.angular.z = atan2(
  //         transformStamped.transform.translation.y,
  //         transformStamped.transform.translation.x);
  // msg.linear.x = scaleForwardSpeed * sqrt(
  //         pow(transformStamped.transform.translation.x, 2) +
  //         pow(transformStamped.transform.translation.y, 2));
    try {
          transformStamped = tf_buffer_->lookupTransform(
            parent_frame_.c_str(), child_frame_.c_str(),
            tf2::TimePointZero);

            T_x = transformStamped.transform.translation.x;
            T_y = transformStamped.transform.translation.y;
            T_z = transformStamped.transform.translation.z;
            O_x = transformStamped.transform.rotation.x;
            O_y = transformStamped.transform.rotation.y;
            O_z = transformStamped.transform.rotation.z;
            O_w = transformStamped.transform.rotation.w;
            // RCLCPP_INFO(
            // this->get_logger(), "tf2 echo %s to %s = {%f, %f}",
            // parent_frame_.c_str(), child_frame_.c_str(), transformStamped.transform.rotation.z , transformStamped.transform.rotation.w);
            // return transformStamped.transform.rotation.z , transformStamped.transform.rotation.w;
        } 
    catch (tf2::TransformException & ex) {
          RCLCPP_INFO(
            this->get_logger(), "Could not transform %s to %s: %s",
            parent_frame_.c_str(), child_frame_.c_str(), ex.what());
            // return 0.0 , 0.0;
        }
}
geometry_msgs::msg::PoseStamped zed2_realignment_gp::nav2_pose(float t_x, float t_y, double theta)
{   
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = t_x;
    goal_pose.pose.position.y = t_y;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation = orientationAroundZAxis(theta);
    
    

    // goal_pose.pose.orientation.x = 0.0;
    // goal_pose.pose.orientation.y = 0.0;
    // goal_pose.pose.orientation.z = o_z;
    // goal_pose.pose.orientation.w = o_w;
    return goal_pose;
}

void zed2_realignment_gp::update()
{
  z2b.clear(); 
}

void zed2_realignment_gp::updateWpNavigationMarkers(std::vector<geometry_msgs::msg::PoseStamped> poses_)
{
    resetUniqueId();
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    for (size_t i = 0; i < poses_.size(); i++)
    {
        visualization_msgs::msg::Marker arrow_marker;
        arrow_marker.header = poses_[i].header;
        arrow_marker.id = getUniqueId();
        arrow_marker.type = visualization_msgs::msg::Marker::ARROW;
        arrow_marker.action = visualization_msgs::msg::Marker::ADD;
        arrow_marker.pose = poses_[i].pose;
        arrow_marker.scale.x = 0.3;
        arrow_marker.scale.y = 0.05;
        arrow_marker.scale.z = 0.02;
        arrow_marker.color.r = 0;
        arrow_marker.color.g = 255;
        arrow_marker.color.b = 0;
        arrow_marker.color.a = 1.0f;
        arrow_marker.lifetime = rclcpp::Duration(0s);
        arrow_marker.frame_locked = false;
        marker_array->markers.push_back(arrow_marker);

        // Draw a red circle at the waypoint pose
        visualization_msgs::msg::Marker circle_marker;
        circle_marker.header = poses_[i].header;
        circle_marker.id = getUniqueId();
        circle_marker.type = visualization_msgs::msg::Marker::SPHERE;
        circle_marker.action = visualization_msgs::msg::Marker::ADD;
        circle_marker.pose = poses_[i].pose;
        circle_marker.scale.x = 0.05;
        circle_marker.scale.y = 0.05;
        circle_marker.scale.z = 0.05;
        circle_marker.color.r = 255;
        circle_marker.color.g = 0;
        circle_marker.color.b = 0;
        circle_marker.color.a = 1.0f;
        circle_marker.lifetime = rclcpp::Duration(0s);
        circle_marker.frame_locked = false;
        marker_array->markers.push_back(circle_marker);

        // Draw the waypoint number
        visualization_msgs::msg::Marker marker_text;
        marker_text.header = poses_[i].header;
        marker_text.id = getUniqueId();
        marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::msg::Marker::ADD;
        marker_text.pose = poses_[i].pose;
        marker_text.pose.position.z += 0.2;  // draw it on top of the waypoint
        marker_text.scale.x = 0.07;
        marker_text.scale.y = 0.07;
        marker_text.scale.z = 0.07;
        marker_text.color.r = 0;
        marker_text.color.g = 255;
        marker_text.color.b = 0;
        marker_text.color.a = 1.0f;
        marker_text.lifetime = rclcpp::Duration(0s);
        marker_text.frame_locked = false;
        marker_text.text = "wp_" + std::to_string(i + 1);
        marker_array->markers.push_back(marker_text);
    }

    if (marker_array->markers.empty()) 
    {
    visualization_msgs::msg::Marker clear_all_marker;
    clear_all_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array->markers.push_back(clear_all_marker);
    }

    wp_navigation_markers_pub_->publish(std::move(marker_array));
    // poses_.clear();

}

void zed2_realignment_gp::resetUniqueId()
{
  unique_id = 0;
}

int zed2_realignment_gp::getUniqueId()
{
  int temp_id = unique_id;
  unique_id += 1;
  return temp_id;
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zed2_realignment_gp>());
  rclcpp::shutdown();
  return 0;
}



