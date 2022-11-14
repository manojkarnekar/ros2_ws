#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <memory>
#include <string>
#include <math.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class DiffTf : public rclcpp::Node
{
  public:
    DiffTf()
    : Node("DiffTf_odom")
    {
      l_whell_encoder = this->create_subscription<std_msgs::msg::Int32>(
      "lwheel_encoder", 10, std::bind(&DiffTf::leftencoderCb, this, _1));

      r_whell_encoder = this->create_subscription<std_msgs::msg::Int32>(
      "rwheel_encoder", 10, std::bind(&DiffTf::rightencoderCb, this, _1));

      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

	  timer_ = this->create_wall_timer(1ms, std::bind(&DiffTf::update, this));

	  odom_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      init_variables();
    }

  private:
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr l_whell_encoder;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr r_whell_encoder;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    void leftencoderCb(const std_msgs::msg::Int32::SharedPtr left_ticks);
    void rightencoderCb(const std_msgs::msg::Int32::SharedPtr right_ticks);
	rclcpp::TimerBase::SharedPtr timer_;

	std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf;

    double encoder_min;
	double encoder_max;
	double encoder_low_wrap;
	double encoder_high_wrap;
	double prev_lencoder;
	double prev_rencoder;
	double lmult;
	double rmult;
	double left;
	double right;
	double rate;
    double enc_left ;
	double enc_right;
	double ticks_meter;
	double base_width;
	double dx, dr;
	double x_final,y_final, theta_final;
	double elapsed;
	double d_left, d_right, d, th,x,y;
	double dt;
	rclcpp::Time t_next, then, current_time, last_time;
	bool odom_tf_pub;

    void init_variables();
	void update();
	

};

void DiffTf::init_variables()
{
    
    prev_lencoder = 0;
	prev_rencoder = 0;
	elapsed = 0;
	d_left = 0; 
	d_right = 0;
	d = 0;
	th = 0;
	x = 0;
	y = 0;
	dt = 0;

	lmult = 0;
	rmult = 0;
	left = 0;
	right = 0;
	encoder_min =  -2147483647;
	encoder_max =  2147483647;
	rate = 10.0;
	ticks_meter = 3185;
	base_width = 0.465;
    encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

	// rclcpp::Time t_delta = rclcpp::Duration(1/rate, 0);
	t_next = this->get_clock()->now() + rclcpp::Duration(1/rate, 0);
	then = this->get_clock()->now();

	current_time = this->get_clock()->now();
	last_time = this->get_clock()->now();


    enc_left = 0;
	enc_right = 0;
	dx = 0;
	dr = 0;
	x_final = 0;y_final=0;theta_final=0;
	odom_tf_pub = false;

}


void DiffTf::update()
{
	rclcpp::Time now = this->get_clock()->now();
	rclcpp::Rate loop_rate(rate);
	if ( now > t_next)
	{
		rclcpp::Duration elapsed = now - then;
		dt = elapsed.seconds();
		// std::cout <<elapsed.seconds()<< std::endl;

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
		then = now;
		loop_rate.sleep();

		if (odom_tf_pub)
		{
			geometry_msgs::msg::TransformStamped tf_odom;
			tf_odom.header.stamp = now;
			tf_odom.header.frame_id = "odom";
			tf_odom.child_frame_id = "base_footprint";

			tf_odom.transform.translation.x = x_final;
			tf_odom.transform.translation.y = y_final;
			tf_odom.transform.translation.z = 0.0;

			tf2::Quaternion q;
			q.setRPY(0, 0, theta_final);
			tf_odom.transform.rotation.x = q.x();
			tf_odom.transform.rotation.y = q.y();
			tf_odom.transform.rotation.z = q.z();
			tf_odom.transform.rotation.w = q.w();

			odom_tf->sendTransform(tf_odom);
		}

	}

	else { ; }

}

void DiffTf::leftencoderCb(const std_msgs::msg::Int32::SharedPtr left_ticks)

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
}

void DiffTf::rightencoderCb(const std_msgs::msg::Int32::SharedPtr right_ticks)

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
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffTf>());
  rclcpp::shutdown();
  return 0;
}