#include "nav2_goal_pose/nav2_goal_pose.h"

void Utility::send_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  // rclcpp::Time now = this->get_clock()->now();
  tf_msg.header.stamp = now;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(th_x, th_y, th_z);
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  target_tf->sendTransform(tf_msg);
}

void Utility::send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  // rclcpp::Time now = this->get_clock()->now();
  tf_msg.header.stamp = now;
  tf_msg.header.frame_id = parent_frame;
  tf_msg.child_frame_id = child_frame;

  tf_msg.transform.translation.x = x;
  tf_msg.transform.translation.y = y;
  tf_msg.transform.translation.z = z;

  tf2::Quaternion q;
  q.setRPY(th_x, th_y, th_z);
  tf_msg.transform.rotation.x = q.x();
  tf_msg.transform.rotation.y = q.y();
  tf_msg.transform.rotation.z = q.z();
  tf_msg.transform.rotation.w = q.w();

  static_target_tf->sendTransform(tf_msg);
}

geometry_msgs::msg::TransformStamped Utility::tf_sub(std::string parent_frame, std::string child_frame)
{
  geometry_msgs::msg::TransformStamped transformStamped;
  transformStamped = tf_buffer_->lookupTransform(
    parent_frame.c_str(), child_frame.c_str(),
    tf2::TimePointZero);

    return transformStamped;
}


