#include "utility.hpp"

using namespace std;
using std::placeholders::_1;
using namespace std::chrono_literals;

class AfsOdom : public rclcpp::Node
{
  public:
    AfsOdom()
    : Node("AfsOdom_node") 
    {
        drive_encoder = this->create_subscription<std_msgs::msg::Int32>("/drive_encoder", 10, std::bind(&AfsOdom::drive_encoder_Cb, this, _1));
        heading_fb = this->create_subscription<std_msgs::msg::Float32>("/heading_fb", 10, std::bind(&AfsOdom::heading_fb_Cb, this, _1));
        odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/afs_odom", 10);
        alpha_odom_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/alphasense/odom", 10);
        odom_tf = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_target_tf = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        // timer_ = this->create_wall_timer(20ms, std::bind(&AfsOdom::update, this));
        odom_tf_pub = false;
    }
  public:
    void heading_fb_Cb(const std_msgs::msg::Float32::SharedPtr msg);
    void drive_encoder_Cb(const std_msgs::msg::Int32::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_encoder;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr heading_fb;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr alpha_odom_pub;
    rclcpp::TimerBase::SharedPtr timer_;

    void init_variables();
    void update();

	double encoder_min_val =  -2147483647;
	double encoder_max_val =  2147483647; // data type Int32 so 2pow(32) = 4294967296   (4294967296/2=2147483647)  
    double ticks_meter = 23468;
    double base_width = 0.3646;
    double base_length = 0.892;
    double encoder_low_wrap = ((encoder_max_val - encoder_min_val) * 0.3) + encoder_min_val ;
	double encoder_high_wrap = ((encoder_max_val - encoder_min_val) * 0.7) + encoder_min_val ;
	double rate = 50.0;
    double motor_direction = 1;
    double encoder_direction = 1;
    double angle ,angle_prev ,dr_enc_prev ,drive_enc ,drive_dist ,dr_mult ,X , Y ,Vx , Vy , Wz ,Vd ,heading,dr_enc,dt,angle_mean;
    double Z = 0.0;
    bool odom_tf_pub;
    double drive_enc_prev;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_target_tf;
    void send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z);
};


void AfsOdom::update()
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
    
    // rclcpp::Time now = this->get_clock()->now();
    // send_static_target_tf(now, "odom", "base_footprint", X, Y, 0.0, 0.0, 0.0, angle);


    // if (odom_tf_pub)
    // {
    //     geometry_msgs::msg::TransformStamped tf_odom;
    //     tf_odom.header.stamp = now;
    //     tf_odom.header.frame_id = "odom";
    //     tf_odom.child_frame_id = "base_footprint";

    //     tf_odom.transform.translation.x = X;
    //     tf_odom.transform.translation.y = Y;
    //     tf_odom.transform.translation.z = 0.0;

    //     // tf2::Quaternion q;
    //     // q.setRPY(0, 0, theta_final);
    //     tf_odom.transform.rotation.x = quaternion[0];
    //     tf_odom.transform.rotation.y = quaternion[1];
    //     tf_odom.transform.rotation.z = quaternion[2];
    //     tf_odom.transform.rotation.w = quaternion[3];

    //     odom_tf->sendTransform(tf_odom);
    // }

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

void AfsOdom::heading_fb_Cb(const std_msgs::msg::Float32::SharedPtr msg)
{
    heading = msg->data;
}
void AfsOdom::drive_encoder_Cb(const std_msgs::msg::Int32::SharedPtr msg)
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

void AfsOdom::send_static_target_tf(rclcpp::Time now, std::string parent_frame, std::string child_frame, float x, float y, float z, float th_x, float th_y, float th_z)
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
  rclcpp::spin(std::make_shared<AfsOdom>());
  rclcpp::shutdown();
  return 0;
}
