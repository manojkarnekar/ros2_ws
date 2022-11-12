#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>

#define RAD2DEG(x) ((x)*180.0/M_PI)
#define DEG2RED(x) ((x)*M_PI/180.0)
// #include <visualization_msgs/msg/marker.hpp>

#include<iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <chrono>
#include <memory>
#include <math.h>

using namespace std;



using std::placeholders::_1;
using namespace std::chrono_literals;



class nav2_goal_set : public rclcpp::Node
{
    public:
        nav2_goal_set()
        : Node("nav2_action_goal_pose")
        {
            this->declare_parameter<std::string>("parent_frame", "map");
            this->get_parameter("parent_frame", parent_frame_);

            this->declare_parameter<std::string>("child_frame", "base_footprint");
            this->get_parameter("child_frame", child_frame_);

            cv_pose_arr = this->create_subscription<std_msgs::msg::Float32MultiArray>(
                "/cv_pose", 10, std::bind(&nav2_goal_set::nav2_goal_pose_callback, this, _1));

            odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 
            rclcpp::SystemDefaultsQoS(), 
            std::bind(&nav2_goal_set::odomCallback, this, std::placeholders::_1));
            
            pose_stamped_pub_L = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_l", 10);
            pose_stamped_pub_M = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_m", 10);
            pose_stamped_pub_R = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_r", 10);
            pose_stamped_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
            
            

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // timer_tf = this->create_wall_timer(1ms, std::bind(&nav2_goal_set::tf_sub, this));

        }
        
    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_L;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_M;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_R;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cv_pose_arr;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

        void nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg);

        void read_goal_pose();
        void nav2_pose(float x, float y, float O_z, float O_w, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub);
        
        float x,y;
        float d_i = 0.0;
        float d_f = 0.0;
        float d_t = 0.55;
        int cnt = 0;

        float x_l, y_l, x_m, y_m, x_r, y_r, T_x, T_y, O_z, O_w;
        std::string parent_frame_;
        std::string child_frame_;
        std::string odom_topic = "/odometry/filtered";
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::TimerBase::SharedPtr timer_tf;
        geometry_msgs::msg::TransformStamped transformStamped;
        // void tf_sub();
        float odom_tx, odom_ty, odom_az, odom_aw;
        
};


void nav2_goal_set::nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // d_i = msg->data;

    float del_r = 0.7;

    x_m = odom_tx + msg->data.at(7);
    y_m = x_m * tan(DEG2RED(odom_az));

    x_l = x_m;
    y_l = -(msg->data.at(6)+ y_m + del_r);

    x_r = x_m;
    y_r =  (msg->data.at(8)+ y_m + del_r);

    // RCLCPP_INFO(
    //         this->get_logger(), "tf2 echo %s to %s = {%f, %f}",
    //         parent_frame_.c_str(), child_frame_.c_str(), O_z, O_w);

    // cout<<"T_x = " <<T_x<<" "<<"T_y = " <<T_y<<" "<<"O_z = " << O_z << " O_w = " << O_w<<" y_m = "<<y_m<< endl;
    
    nav2_pose(x_l, y_l, 0.0, 1.0, pose_stamped_pub_L);
    nav2_pose(x_m, y_m, 0.0 , 1.0 , pose_stamped_pub_M);
    nav2_pose(x_r, y_r, 0.0, 1.0, pose_stamped_pub_R);

    if(cnt<1)
    {
        float d = (d_i-d_t);
        nav2_pose(x_r, y_r, 0.0, 1.0, pose_stamped_pub);
        cout<<"distance to travel->>"<<d<< std::endl;
        
    }
    cnt +=1;

}





void nav2_goal_set::nav2_pose(float x, float y, float O_z, float O_w, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub)

{   

    geometry_msgs::msg::PoseStamped goal_poses;
    goal_poses.header.stamp = this->now();
    goal_poses.header.frame_id = "map";
    goal_poses.pose.position.x = x;
    goal_poses.pose.position.y = y;
    goal_poses.pose.position.z = 0.0;
    goal_poses.pose.orientation.x = 0.0;
    goal_poses.pose.orientation.y = 0.0;
    goal_poses.pose.orientation.z = O_z;
    goal_poses.pose.orientation.w = O_w;
    pose_stamped_pub->publish(goal_poses);
}


void nav2_goal_set::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)

{
    auto current_time = rclcpp::Time(msg->header.stamp);
    odom_tx = msg->pose.pose.position.x;
    odom_ty = msg->pose.pose.position.y;
    odom_az = msg->pose.pose.orientation.z;
    odom_aw = msg->pose.pose.orientation.w;
    cout<<"T_x = " <<odom_tx<<" "<<"T_y = " <<odom_ty<<" "<<"O_z = " << odom_az<< " O_w = " << odom_aw<< endl;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_goal_set>());
  rclcpp::shutdown();
  return 0;
}
