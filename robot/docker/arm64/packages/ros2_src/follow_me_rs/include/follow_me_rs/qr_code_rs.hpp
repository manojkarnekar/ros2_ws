#define _USE_MATH_DEFINES

#include <iostream>
#include<fstream>
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

#include <eigen3/Eigen/SVD>
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Dense>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
// #include "example.hpp"         

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