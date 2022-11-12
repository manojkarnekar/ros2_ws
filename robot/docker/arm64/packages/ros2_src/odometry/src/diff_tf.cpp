
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

using std::placeholders::_1;
using namespace std::chrono_literals;

class Core0_odom : public rclcpp::Node
{
  public:
    Core0_odom()
    : Node("Core0_odom_odom")
    {
        this->declare_parameter("odom_tf_pub", true);
        this->declare_parameter("rate_hz", 10.0);
        this->declare_parameter("ticks_meter", 664.0);
        this->declare_parameter("base_width", 0.470);
        this->declare_parameter("base_frame_id", "base_footprint");
        this->declare_parameter("odom_frame_id", "odom");
        this->declare_parameter("encoder_min", -32768);
        this->declare_parameter("encoder_max",  32768);


        odom_tf_pub = get_parameter("odom_tf_pub").as_bool();
        rate_hz = get_parameter("rate_hz").as_double();
        ticks_meter = get_parameter("ticks_meter").as_double();
        base_width = get_parameter("base_width").as_double();
        base_frame_id = get_parameter("base_frame_id").as_string ();
        odom_frame_id = get_parameter("odom_frame_id").as_string ();
        encoder_min = get_parameter("encoder_min").as_double();
        encoder_max = get_parameter("encoder_max").as_double();


        l_whell_encoder = this->create_subscription<std_msgs::msg::Int16>(
        "lwheel_encoder", 10, std::bind(&Core0_odom::leftencoderCb, this, _1));

        r_whell_encoder = this->create_subscription<std_msgs::msg::Int16>(
        "rwheel_encoder", 10, std::bind(&Core0_odom::rightencoderCb, this, _1));

        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

        timer_ = this->create_wall_timer(20ms, std::bind(&Core0_odom::update, this));

        static_target_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    }

  private:
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr l_whell_encoder;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr r_whell_encoder;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

    void leftencoderCb(const std_msgs::msg::Int16::SharedPtr left_ticks);
    void rightencoderCb(const std_msgs::msg::Int16::SharedPtr right_ticks);
	rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_target_tf;
    void send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, double x, double y, double z, double th_x, double th_y, double th_z);
    bool odom_tf_pub;
    double rate_hz, ticks_meter, base_width, encoder_min, encoder_max;
    std::string base_frame_id, odom_frame_id;

    double encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min ;
    double encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min ;

    double enc_left = 0.0;  
    double enc_right = 0.0;
    double left = 0.0;  
    double right = 0.0;
    double lmult = 0.0;
    double rmult = 0.0;
    double prev_lencoder = 0.0;
    double prev_rencoder = 0.0;
    double x = 0.0;  
    double y = 0.0;
    double th = 0.0;
    double dx = 0.0; 
    double dr = 0.0;
    double enc = 0.0;
    double dt = 0.0;
    double x_final = 0.0;
    double y_final = 0.0;
    double theta_final = 0.0;
    double d_left = 0.0;
    double d_right = 0.0;
    double d = 0.0;
    rclcpp::Time now = this->get_clock()->now();
    rclcpp::Time then = this->get_clock()->now();
	void update();
	

};

void Core0_odom::update()
{
    now = this->get_clock()->now();
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
		x = cos(th) * d;
		y = -sin(th) * d;
		// calculate the final position of the robot
		x_final = x_final + ( cos(theta_final) * x - sin(theta_final) * y );
		y_final = y_final + ( sin(theta_final) * x + cos(theta_final) * y );
		// std::cout << "l_&_r wheel encoder subscribing"<<" x_final "<<x_final<<" y_final "<<y_final<<std::endl;
	}

	if( th != 0)
	{
		theta_final = theta_final + th;
		// std::cout << "theta_final encoder subscribing"<<" th_final "<<theta_final<<std::endl;
	}

    geometry_msgs::msg::Quaternion odom_quat;

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

}

void Core0_odom::send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, double x, double y, double z, double th_x, double th_y, double th_z)
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

void Core0_odom::leftencoderCb(const std_msgs::msg::Int16::SharedPtr left_ticks)
{
    enc = left_ticks->data;
    if(enc < encoder_low_wrap && prev_lencoder > encoder_high_wrap)
    {
        lmult = lmult + 1;
    }

    if(enc > encoder_high_wrap && prev_lencoder < encoder_low_wrap)
    {
        lmult = lmult - 1;
    }

    left = 1.0 * (enc + lmult * (encoder_max - encoder_min));
    prev_lencoder = enc;
}

void Core0_odom::rightencoderCb(const std_msgs::msg::Int16::SharedPtr right_ticks)
{
    enc = right_ticks->data;
    if(enc < encoder_low_wrap && prev_rencoder > encoder_high_wrap)
    {
        rmult = rmult + 1;
    }

    if(enc > encoder_high_wrap && prev_rencoder < encoder_low_wrap)
    {
        rmult = rmult - 1;
    }
    
    right = 1.0 * (enc + rmult * (encoder_max - encoder_min));
    prev_rencoder = enc;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Core0_odom>());
  rclcpp::shutdown();
  return 0;
}