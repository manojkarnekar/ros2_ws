#include "odometry/afs_odom.h"

void Afs_Odom::update()
{
	rclcpp::Rate loop_rate(rate);

    if(drive_enc_prev == 0)
    {
        drive_enc_prev = 0;
    }
    else
    {
        drive_dist = encoder_direction * ((drive_enc - drive_enc_prev)/ticks_meter);
    }

    angle = angle_prev + (drive_dist * sin(heading)/base_length);
    angle_mean = 0.5 * (angle + angle_prev);

    X += drive_dist*cos(heading)*cos(angle_mean);
    Y += drive_dist*cos(heading)*sin(angle_mean);

    dt= 1/rate;
    Vd = drive_dist/dt;
    Vx = Vd * cos(heading);
    Wz = Vd * sin(heading) / base_length;
    
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, angle);

    rclcpp::Time now = this->get_clock()->now();
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x = X;
    odom.pose.pose.position.y = Y;
    odom.pose.pose.position.z = 0.0;
    // odom.pose.pose.orientation = quaternion;
    odom.pose.pose.orientation.x = quaternion[0];
    odom.pose.pose.orientation.y = quaternion[1];
    odom.pose.pose.orientation.z = quaternion[2];
    odom.pose.pose.orientation.w = quaternion[3];
    odom.pose.covariance[0]  = 0.01;
	odom.pose.covariance[7]  = 0.01;
	odom.pose.covariance[14] = 99999;
	odom.pose.covariance[21] = 99999;
	odom.pose.covariance[28] = 99999;
	odom.pose.covariance[35] = 0.01;
    odom.child_frame_id = "base_footprint";
    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = Wz;
    odom.twist.covariance = odom.pose.covariance;
    odom_pub->publish(odom);
    
    if (odom_tf_pub)
    {
    send_static_target_tf(now, "odom", "base_footprint", X, Y, 0.0, 0.0, 0.0, angle);
    }

    geometry_msgs::msg::TwistStamped twist_stamp;
    twist_stamp.header.stamp = now;
    twist_stamp.header.frame_id = "odom";
    twist_stamp.twist.linear.x = Vx;
    twist_stamp.twist.linear.y = 0.0;
    twist_stamp.twist.angular.z = Wz;
    alpha_odom_pub->publish(twist_stamp);

    angle_prev = angle;
    drive_enc_prev = drive_enc;

}

void Afs_Odom::heading_fb_Cb(const std_msgs::msg::Float32::SharedPtr msg)
{
    heading = msg->data;
}
void Afs_Odom::drive_encoder_Cb(const std_msgs::msg::Int32::SharedPtr msg)
{
    dr_enc = msg->data;
    
    if(dr_enc < encoder_low_wrap && dr_enc_prev > encoder_high_wrap)
    {
        dr_mult = dr_mult + 1;
    }
    if(dr_enc >encoder_high_wrap && dr_enc_prev <encoder_low_wrap)
    {
        dr_mult =dr_mult - 1;
    }   
    drive_enc = 1.0 * (dr_enc +dr_mult * (encoder_max_val -encoder_min_val));
    dr_enc_prev = dr_enc;
    
    update();

}

void Afs_Odom::send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z)
{
  geometry_msgs::msg::TransformStamped tf_msg;
//   rclcpp::Time now = this->get_clock()->now();
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Afs_Odom>());
  rclcpp::shutdown();
  return 0;
}