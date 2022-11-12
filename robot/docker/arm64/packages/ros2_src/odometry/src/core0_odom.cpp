#include "odometry/core0_odom.h"

void Core0_odom::update()
{
	rclcpp::Time now = this->get_clock()->now();
	rclcpp::Rate loop_rate(rate);
	rclcpp::Duration elapsed = now - then;
	dt = elapsed.seconds();
	
	if(enc_left == 0)
	{
		d_left = 0;
		d_right = 0;
	}
	else
	{
		d_left = (left - enc_left) / ( ticks_meter);
		d_right = (right - enc_right) / ( ticks_meter);
	}
	
	enc_left = left;
	enc_right = right;

	d = (d_left + d_right ) / 2.0;
	th = ( d_right - d_left ) / base_width;

	dx = d/dt;
	dr = th/dt;

	if ( d != 0)
	{
		x = cos( th ) * d;
		y = -sin( th ) * d;
		// calculate the final position of the robot
		x_final = x_final + ( cos( theta_final ) * x - sin( theta_final ) * y );
		y_final = y_final + ( sin( theta_final ) * x + cos( theta_final ) * y );
		// std::cout << "l_&_r wheel encoder subscribing"<<" x_final "<<x_final<<" y_final "<<y_final<<std::endl;
	}

	if( th != 0)
	{
		theta_final = theta_final + th;
		// std::cout << "theta_final encoder subscribing"<<" th_final "<<theta_final<<std::endl;
	}
	

	geometry_msgs::msg::Quaternion odom_quat;
	// tf2::Quaternion odom_quat;
	// odom_quat.setRPY(0.0, 0.0, theta_final);
	odom_quat.x = 0.0;
	odom_quat.y = 0.0;
	odom_quat.z = 0.0;
	odom_quat.z = sin( theta_final / 2 );	
	odom_quat.w = cos( theta_final / 2 );


	nav_msgs::msg::Odometry odom;

	odom.header.stamp = now;
	odom.header.frame_id = "odom";
	
	//set the position
	odom.pose.pose.position.x = x_final;
	odom.pose.pose.position.y = y_final;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	odom.pose.covariance[0]  = 0.01;
	odom.pose.covariance[7]  = 0.01;
	odom.pose.covariance[14] = 99999;
	odom.pose.covariance[21] = 99999;
	odom.pose.covariance[28] = 99999;
	odom.pose.covariance[35] = 0.01;

	//set the velocity
	odom.child_frame_id = "base_footprint";
	odom.twist.twist.linear.x = dx;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = dr;
	odom.twist.covariance = odom.pose.covariance;
	//publish the message
	odom_pub->publish(odom);
	
	// loop_rate.sleep();
	if (odom_tf_pub)
	{
		send_static_target_tf(now, "odom", "base_footprint", x_final, y_final, 0.0, 0.0, 0.0, theta_final);
	}
	then = now;


}

void Core0_odom::leftencoderCb(const std_msgs::msg::Int16::SharedPtr left_ticks)

{
	double enc = left_ticks->data;
	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	{
		lmult = lmult + 1;
	}
	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))
	{
        lmult = lmult - 1;
	}
	left = 1.0 * (enc + lmult * (encoder_max - encoder_min ));
	// std::cout <<" left_distance "<<left<<std::endl;
	prev_lencoder = enc;

	update();
	// RCLCPP_INFO(get_logger(), "getting l_encoder data...");
	// update();
}

void Core0_odom::rightencoderCb(const std_msgs::msg::Int16::SharedPtr right_ticks)

{
	double enc = right_ticks->data;
	if((enc < encoder_low_wrap) && (prev_lencoder > encoder_high_wrap))
	{
		rmult = rmult + 1;
	}
	if((enc > encoder_high_wrap) && (prev_lencoder < encoder_low_wrap))
	{
		rmult = rmult - 1;
	}
	right = 1.0 * (enc + rmult * (encoder_max - encoder_min ));
	// std::cout <<" right_distance "<<right<<std::endl;
	prev_rencoder = enc;
	// RCLCPP_INFO(get_logger(), "getting r_encoder data...");
}

void Core0_odom::send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z)
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
  rclcpp::spin(std::make_shared<Core0_odom>());
  rclcpp::shutdown();
  return 0;
}