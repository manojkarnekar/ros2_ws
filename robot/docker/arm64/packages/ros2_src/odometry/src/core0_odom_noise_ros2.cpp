// Include various libraries
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
 
using namespace std;

using std::placeholders::_1;
using namespace std::chrono_literals;

class Core0_odom : public rclcpp::Node
{
  public:
    Core0_odom()
    : Node("Core0_odom_odom")
    {
      // this->declare_parameter<std::string>("odom_tf_pub", "false");
      l_whell_encoder = this->create_subscription<std_msgs::msg::Int16>(
      "lwheel_encoder", 100, std::bind(&Core0_odom::leftencoderCb, this, _1));

      r_whell_encoder = this->create_subscription<std_msgs::msg::Int16>(
      "rwheel_encoder", 100, std::bind(&Core0_odom::rightencoderCb, this, _1));

      odom_data_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", 100);
      odom_data_pub_quat = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
      

      timer_ = this->create_wall_timer(20ms, std::bind(&Core0_odom::update, this));
    }
    

  private:
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr l_whell_encoder;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr r_whell_encoder;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat;

    void leftencoderCb(const std_msgs::msg::Int16::SharedPtr left_ticks);
    void rightencoderCb(const std_msgs::msg::Int16::SharedPtr right_ticks);

	rclcpp::TimerBase::SharedPtr timer_;
    
    // Robot physical constants
    const double TICKS_PER_REVOLUTION = 313; // For reference purposes.
    const double WHEEL_RADIUS = 0.075; // Wheel radius in meters
    const double WHEEL_BASE = 0.47; // Center of left tire to center of right tire
    const double TICKS_PER_METER = 664; // Original was 2800

    // Initial pose
    const double initialX = 0.0;
    const double initialY = 0.0;
    const double initialTheta = 0.00000000001;
    const double PI = 3.141592;
    
    // Distance both wheels have traveled
    double distanceLeft = 0;
    double distanceRight = 0;

    nav_msgs::msg::Odometry odomNew;
    nav_msgs::msg::Odometry odomOld;
	
    // Set the data fields of the odometry message
    void update();
    void publish_quat();
    void update_odom();

    rclcpp::Time then = this->get_clock()->now();
    double dt = 0.0;

};
 
// Calculate the distance the left wheel has traveled since the last cycle
void Core0_odom::leftencoderCb(const std_msgs::msg::Int16::SharedPtr leftCount){
 
  static int lastCountL = 0;
  if(leftCount->data != 0 && lastCountL != 0) {
         
    int leftTicks = (leftCount->data - lastCountL);
 
    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    }
    else if (leftTicks < -10000) {
      leftTicks = 65535-leftTicks;
    }
    else{}
    distanceLeft = leftTicks/TICKS_PER_METER;
  }
  lastCountL = leftCount->data;
}
 
// Calculate the distance the right wheel has traveled since the last cycle
void Core0_odom::rightencoderCb(const std_msgs::msg::Int16::SharedPtr rightCount)
{
  static int lastCountR = 0;
  if(rightCount->data != 0 && lastCountR != 0) {
 
    int rightTicks = rightCount->data - lastCountR;
     
    if (rightTicks > 10000) {
      distanceRight = (0 - (65535 - distanceRight))/TICKS_PER_METER;
    }
    else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    else{}
    distanceRight = rightTicks/TICKS_PER_METER;
  }
  lastCountR = rightCount->data;
}
 
// Publish a nav_msgs::msg::Odometry message in quaternion format
void Core0_odom::publish_quat() {
 
  tf2::Quaternion q;
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
 
  nav_msgs::msg::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;

  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
 
  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }
 
  odom_data_pub_quat->publish(quatOdom);
}
 
// Update odometry information
void Core0_odom::update_odom() {
 
  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;

  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);
 
  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
     
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}
 
  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;
 
  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }
 
  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}
 
  // Compute the velocity
  rclcpp::Time now = this->get_clock()->now();
  rclcpp::Duration elapsed = now - then;
  dt = elapsed.seconds();

  odomNew.header.stamp = now;
  odomNew.twist.twist.linear.x = cycleDistance/(dt);
  odomNew.twist.twist.angular.z = cycleAngle/(dt);
 
  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;
 
  // Publish the odometry message
  odom_data_pub->publish(odomNew);
  then = now;
}

void Core0_odom::update()
{
    update_odom();
    publish_quat();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Core0_odom>());
  rclcpp::shutdown();
  return 0;
}