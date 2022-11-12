#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>
// #include <geometry_msgs/msg/PoseStamped.h>
#include <cmath> 
#include <limits>

using namespace std;
using std::placeholders::_1;

class QRFollower : public rclcpp::Node
{
  public:
    QRFollower()
    : Node("QRFollower")

    {
        QRSub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "qr_pose", 10, std::bind(&QRFollower::QRcodeCb, this, _1));
        
        cmdPub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel" ,rclcpp::QoS(rclcpp::KeepLast(10)));

        // scanSub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS(), std::bind(&QRFollower::scanCb, this, _1));
    }

	public:

        rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr QRSub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdPub_;
        // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scanSub;

        void scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void QRcodeCb(const std_msgs::msg::Float32MultiArray::SharedPtr qr_pose);
        float Distance(float x, float y);
        void follow(float x, float z);
        void stop(float error);
        void slowdown();

		double stopDistance_ = 0.5; //margin of space between powerbot and QRcode being followed, for safety
		double speed_ = 0.26; //constant to scale speed by (proportional control)
		double iGain_ = 0.3; //constant to scale turning by (integral control)
		double pGain_ = 0.3; //constant to scale turning by (proportional control)
		double maxLinearVel_ = 0.40; 
		double maxAngularVel = 0.8; 

		float prevX_ = 0; //store past linear velocity x
		float prevZ_ = 0; //store past Angular velocity z
		float noQRDetection_ = -1; //-1 when nothing detected, the point which is detected by laser scan
		float CameraDistance = 0.0; //distance between camera and front of Powerbot, along x-axis of Powerbot
};

		// void QRFollower::scanCb(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
		// 	noQRDetection_ = -1;
		// 	for (auto val : scan->ranges) {
		// 		if (!std::isnan(val)) {
		// 			if (val < noQRDetection_ || noQRDetection_ < 0) {
		// 				noQRDetection_ = val;
		// 			}
		// 		}
		// 	}
		// 	//call stop if there is obstacle which is closer than our set stopDistance_
		// 	if (noQRDetection_ < stopDistance_) {
		// 		stop(noQRDetection_ - stopDistance_);
		// 		// ROS_ERROR("Laser detects something too close, error: %.3f", noDetection_ - stopDistance_);
		// 	}
		// }

        void QRFollower::QRcodeCb(const std_msgs::msg::Float32MultiArray::SharedPtr qr_pose)
		{
			float a_x = qr_pose->data.at(0);
            float a_z = qr_pose->data.at(2);

			float currentDistance = Distance(a_x, a_z);

			if (currentDistance > stopDistance_)
			{
                        	follow(a_x, a_z);
                                // rclcpp::spin();
            }

			else //if (currentDistance < stopDistance_ + CameraDistance)	//detected QRcode is too close, don't follow it
                        {
                            cout << "stop initiated" << endl;
				            stop(currentDistance - stopDistance_ - CameraDistance);
                        }
                        
			// else if (noQRDetection_ > stopDistance_)
            //             {   
            //                 cout << "slowdown" << endl;
			// 	            slowdown();
            //             }
                //  ROS_DEBUG("Hello %s", "World");

		}

		float QRFollower::Distance(float x, float y)
		{
			return sqrt(x*x + y*y);
		}



		void QRFollower::follow(float x, float z)
		{
			geometry_msgs::msg::Twist cmd;                 //(new geometry_msgs::msg::Twist());
			float currentDistance = Distance(x, z);
            cout << "distance = " << currentDistance << endl;
			cmd.linear.x = (currentDistance - stopDistance_ - CameraDistance) * speed_; //catch up to person
			cmd.angular.z = iGain_ * atan(x / z) + pGain_ * (currentDistance - stopDistance_ - CameraDistance);

			//limit velocity commands to max
			if (cmd.linear.x > maxLinearVel_)
				cmd.linear.x = maxLinearVel_;
			else if (cmd.linear.x < -1 * maxLinearVel_)
				cmd.linear.x = -1 * maxLinearVel_;

			if (cmd.angular.z > maxAngularVel)
				cmd.angular.z = maxAngularVel;
			else if (cmd.angular.z < -1 * maxAngularVel)
				cmd.angular.z = -1 * maxAngularVel;

			cmdPub_->publish(cmd);
			prevX_ = cmd.linear.x;
			prevZ_ = cmd.angular.z;
		}


		void QRFollower::stop(float error)
		{
			geometry_msgs::msg::Twist cmd;              //(new geometry_msgs::msg::Twist());
			if (fabs(error) < stopDistance_ / 10)
                        {
				//error is a small fraction of the safety distance, stop moving
				cmd.linear.x = 0;
				cmd.angular.z = 0;
			}
			else
                        {
				//otherwise, move backwards to make space
				cmd.linear.x = 0;
				cmd.angular.z = 0;
			}

			cmdPub_->publish(cmd);
			prevX_ = cmd.linear.x;
			prevZ_ = cmd.angular.z;
		}


		void QRFollower::slowdown()
        {
			geometry_msgs::msg::Twist cmd;             // (new geometry_msgs::msg::Twist());
			cmd.linear.x = prevX_ / 100;
			cmd.angular.z = 0;
			cmdPub_->publish(cmd);

			prevX_ = cmd.linear.x;
			prevZ_ = cmd.angular.z;
		}


        int main(int argc, char * argv[])
        {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<QRFollower>());
        rclcpp::shutdown();
        return 0;
        }
	


