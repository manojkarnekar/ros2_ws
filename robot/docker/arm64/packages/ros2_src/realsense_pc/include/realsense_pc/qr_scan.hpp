#define _USE_MATH_DEFINES

#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <bits/stdc++.h>
#include <time.h>
#include <memory>
#include <algorithm>
#include <vector>
#include <zbar.h>
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <armadillo>
#include <cmath>


#include <chrono>
#include <string>
#include <fstream>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/int16.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "sensor_msgs/msg/laser_scan.hpp"
#include <tf2/LinearMath/Matrix3x3.h>

#include <eigen3/Eigen/SVD>
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Dense>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API

// #include "example.hpp"          // Include short list of convenience functions for rendering

// #include <sl/Camera.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/cvconfig.h>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std;
using namespace cv;
using namespace zbar;
using namespace Eigen;
using pixel = std::pair<int, int>;


typedef struct
{
  string type;
  string data;
  vector <Point> location;
} decodedObject;