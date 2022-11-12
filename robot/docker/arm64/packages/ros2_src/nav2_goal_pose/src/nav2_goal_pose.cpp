#include "nav2_goal_pose/nav2_goal_pose.h"

geometry_msgs::msg::PoseStamped Utility::nav2_pose(geometry_msgs::msg::TransformStamped tf_msg)
{   
    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = tf_msg.transform.translation.x;
    goal_pose.pose.position.y = tf_msg.transform.translation.y;
    goal_pose.pose.position.z = tf_msg.transform.translation.z;
    // goal_pose.pose.orientation = orientationAroundZAxis(theta);
    goal_pose.pose.orientation.x = tf_msg.transform.rotation.x;
    goal_pose.pose.orientation.y = tf_msg.transform.rotation.y;
    goal_pose.pose.orientation.z = tf_msg.transform.rotation.z;
    goal_pose.pose.orientation.w = tf_msg.transform.rotation.w;
    return goal_pose;
}

void Utility::bed_ai(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  rclcpp::Time now = this->get_clock()->now();
  vector<double> co = Coor_find(msg);

  send_static_target_tf(now, camera_frame, bed_frameA, co.at(0),0.0,co.at(1), 0.0, 0.0, 0.0);
  send_static_target_tf(now, camera_frame, bed_frameM, co.at(2),0.0,co.at(3), 0.0, 0.0, 0.0);
  send_static_target_tf(now, camera_frame, bed_frameC, co.at(4),0.0,co.at(5), 0.0, 0.0, 0.0);
}

void Utility::nav2_goal_pose_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  bed_ai(msg);
}

