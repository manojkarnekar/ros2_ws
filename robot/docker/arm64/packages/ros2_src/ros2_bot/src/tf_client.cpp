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
// #include <visualization_msgs/msg/marker.hpp>

#include<iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <map>
#include <chrono>
#include <memory>


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
            
            pose_stamped_pub_L = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_l", 10);
            pose_stamped_pub_M = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_m", 10);
            pose_stamped_pub_R = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose_r", 10);

            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            timer_tf = this->create_wall_timer(1ms, std::bind(&nav2_goal_set::tf_sub, this));


            
        }
        
    private:
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_L;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_M;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub_R;

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cv_pose_arr;
        void nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

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
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::TimerBase::SharedPtr timer_tf;
        geometry_msgs::msg::TransformStamped transformStamped;

        void tf_sub();

        
};

void nav2_goal_set::tf_sub()
{
    try {
          transformStamped = tf_buffer_->lookupTransform(
            parent_frame_.c_str(), child_frame_.c_str(),
            tf2::TimePointZero);

            T_x = transformStamped.transform.translation.x;
            T_y = transformStamped.transform.translation.y;
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


void nav2_goal_set::nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    // d_i = msg->data;

    float del_r = 0.55;
    // x_l = msg->data.at(3);
    // y_l = -(msg->data.at(2)+msg->data.at(6) + del_r);

    x_m = T_x + msg->data.at(3);
    y_m = T_y + msg->data.at(2);

    // x_r = msg->data.at(3);
    // y_r = (msg->data.at(2)+msg->data.at(7) + del_r);

    
    // RCLCPP_INFO(
    //         this->get_logger(), "tf2 echo %s to %s = {%f, %f}",
    //         parent_frame_.c_str(), child_frame_.c_str(), O_z, O_w);
    cout<<"T_x = " <<T_x<<" "<<"T_y = " <<T_y<<" "<<"O_z = " << O_z << " O_w = " << O_w << endl;

    // nav2_pose(x_l, y_l, O_z, O_w, pose_stamped_pub_L);
    nav2_pose(x_m, y_m, O_z, O_w, pose_stamped_pub_M);
    // nav2_pose(x_r, y_r, O_z, O_w, pose_stamped_pub_R);

    // if(cnt<1)
    // {
    //     float d = (d_i-d_t);
    //     cout<<"distance to travel->>"<<d<< std::endl;
        
    // }
    // cnt +=1;

}





void nav2_goal_set::nav2_pose(float x, float y, float O_z, float O_w, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_pub)

{   

    geometry_msgs::msg::PoseStamped goal_poses;
    goal_poses.header.stamp = this->now();
    goal_poses.header.frame_id = "base_footprint";
    goal_poses.pose.position.x = x;
    goal_poses.pose.position.y = y;
    goal_poses.pose.position.z = 0.0;
    goal_poses.pose.orientation.x = 0.0;
    goal_poses.pose.orientation.y = 0.0;
    goal_poses.pose.orientation.z = O_z;
    goal_poses.pose.orientation.w = O_w;
    pose_stamped_pub->publish(goal_poses);
}




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<nav2_goal_set>());
  rclcpp::shutdown();
  return 0;
}
