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


#include <sl/Camera.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/cvconfig.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using namespace std;
// using namespace cv;
using namespace zbar;
using namespace sl;

typedef struct
{
  std::string type;
  std::string data;
  std::vector <cv::Point> location;
}decodedObject;

class QR_code_detection : public rclcpp::Node
{
  public:
    QR_code_detection()
    : Node("QR_code_detection")
    { 
        qr_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("qr_pose", 10);
        img_pose_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("img_pose", 10);
        qr_data_pub = this->create_publisher<std_msgs::msg::String>("qr_data", 10);
    }
    public:
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr qr_pose_pub;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr img_pose_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr qr_data_pub;
        cv::Mat slMat2cvMat(Mat& input);
        int getOCVtype(sl::MAT_TYPE type);
        bool pix_pcl(int x, int y, sl::Mat point_cloud, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub);
        void decode(cv::Mat &im, vector<decodedObject>&decodedObjects);
        void display(cv::Mat &im, vector<decodedObject>&decodedObjects, sl::Mat point_cloud, sl::Mat image);
        // vector<float> dis;
        vector<float> minmax_idx(vector<float>& v);
        void Condition(cv::Mat &im, int x, int y, sl::Mat point_cloud, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub);

        // int zed_init();       

};

cv::Mat QR_code_detection::slMat2cvMat(Mat& input)
{
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

int QR_code_detection::getOCVtype(sl::MAT_TYPE type)
{
    int cv_type = -1;
    switch (type) {
        case MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

bool QR_code_detection::pix_pcl(int x, int y, sl::Mat point_cloud, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub)
{
    sl::float4 point_cloud_value;
    point_cloud.getValue(x, y, &point_cloud_value);

    std_msgs::msg::Float32MultiArray pose_msg;

    if(std::isfinite(point_cloud_value.z))
    {
        pose_msg.data.push_back(point_cloud_value.x/1000.0);
        pose_msg.data.push_back(point_cloud_value.y/1000.0);
        pose_msg.data.push_back(point_cloud_value.z/1000.0);

        pose_pub->publish(pose_msg);

        float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
        cout<<"Distance to Camera at {"<<x<<";"<<y<<"}: "<<distance<<"mm"<<endl;
        return true;
    }
    else
    {
        return false;
    }
}

vector<float> QR_code_detection::minmax_idx(vector<float>& v)
{
  auto it = std::minmax_element(v.begin(), v.end());
	float min = *it.first;
	// float max = *it.second;
  float i = it.first-v.begin();

  vector<float> vec {};
  vec.push_back(min);
  vec.push_back(i);
  return vec;
}

void QR_code_detection::decode(cv::Mat &im, vector<decodedObject>&decodedObjects)
{
  ImageScanner scanner;
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
  scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
  cv::Mat imGray;
  cv::cvtColor(im, imGray, cv::COLOR_BGR2GRAY);
  zbar::Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

  int n = scanner.scan(image);

  // Print results
  for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
  {
    decodedObject obj;

    obj.type = symbol->get_type_name();
    obj.data = symbol->get_data();

    // Print type and data
    cout << "Type : " << obj.type << endl;
    cout << "Data : " << obj.data << endl << endl;

    auto std_message = std_msgs::msg::String();
    std_message.data = obj.data;

    qr_data_pub->publish(std_message);

    // Obtain location
    for(int i = 0; i< symbol->get_location_size(); i++)
    {
      obj.location.push_back(cv::Point(symbol->get_location_x(i),symbol->get_location_y(i)));
    }

    decodedObjects.push_back(obj);
  }
}

void QR_code_detection::Condition(cv::Mat &im, int x, int y, sl::Mat point_cloud, rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pose_pub)
{
    if (pix_pcl(x, y, point_cloud, pose_pub) == true)
    {
        cv::circle(im, cv::Point(x , y), 5, cv::Scalar(0,255,0),-1, 1,0);
    }

    else
    {
        for(long int i=0; i<=15; i++)
        {
            if (pix_pcl(x+i , y+i, point_cloud, pose_pub) == true)
            {
                cv::circle(im, cv::Point(x+i , y+i), 5, cv::Scalar(0,255,0),-1, 1,0);
            }

            else
            {
                if (pix_pcl(x+i , y-i, point_cloud, pose_pub) == true)
                {
                    cv::circle(im, cv::Point(x+i , y-i), 5, cv::Scalar(0,255,0),-1, 1,0);
                }

                else
                {
                    if (pix_pcl(x-i , y+i, point_cloud, pose_pub) == true)
                    {
                        cv::circle(im, cv::Point(x-i , y+i), 5, cv::Scalar(0,255,0),-1, 1,0);
                    }
                    else
                    {
                        if (pix_pcl(x-i , y-i, point_cloud, pose_pub) == true)
                        {
                            cv::circle(im, cv::Point(x-i , y-i), 5, cv::Scalar(0,255,0),-1, 1,0);
                        }
                        else
                        {
                            cout<<"The Distance can not be computed by ZED2 PCL"<<endl;
                        }
                    }

                }
            }

        }

    }


}

void QR_code_detection::display(cv::Mat &im, vector<decodedObject>&decodedObjects, sl::Mat point_cloud, sl::Mat image)
{
  // Loop over all decoded objects
  for(int i = 0; i < decodedObjects.size(); i++)
  {
    vector<cv::Point> points = decodedObjects[i].location;
    vector<cv::Point> hull;

    // If the points do not form a quad, find convex hull
    if(points.size() > 4)
      cv::convexHull(points, hull);
    else
      hull = points;
    

    // Number of points in the convex hull
    int n = hull.size();

    for(int j = 0; j < n; j++)
    {
      line(im, hull[j], hull[ (j+1) % n], cv::Scalar(255,0,0), 3);
      // cout<<hull[j]<<" ";
    }
    // cout<<endl;

    cv::circle(im, cv::Point((hull[0], hull[1])), 5, cv::Scalar(0,255,0),-1, 1,0);
    cv::circle(im, cv::Point((hull[1], hull[0])), 5, cv::Scalar(0,255,0),-1, 1,0);
    cv::circle(im, cv::Point((hull[2], hull[3])), 5, cv::Scalar(0,255,0),-1, 1,0);
    cv::circle(im, cv::Point((hull[3], hull[2])), 5, cv::Scalar(0,255,0),-1, 1,0);

    // left mid point
    cv::circle(im, cv::Point(((hull[0]+hull[1])/2.0, (hull[1]+hull[0])/2.0)), 5, cv::Scalar(0,255,0),-1, 1,0);

    // right mid point
    cv::circle(im, cv::Point(((hull[2]+hull[3])/2.0, (hull[3]+hull[2])/2.0)), 5, cv::Scalar(0,255,0),-1, 1,0);

    //mid point
    cv::Point pt = ((hull[0]+hull[1]+hull[2]+hull[3])/4.0 , (hull[1]+hull[0] + hull[3]+hull[2])/4.0);
    int im_x = image.getWidth() / 2;
    int im_y = image.getHeight() / 2;

    Condition(im, pt.x, pt.y, point_cloud, qr_pose_pub);
    
    Condition(im, im_x, im_y, point_cloud, img_pose_pub);
  }

}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  sl::Camera zed;
  sl::InitParameters init_parameters;
  init_parameters.camera_resolution = sl::RESOLUTION::HD1080;
  init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE; // Use PERFORMANCE depth mode
  init_parameters.coordinate_units = sl::UNIT::MILLIMETER;

  // Open the camera
  auto returned_state = zed.open(init_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) 
  {
    std::cout << "Error " << returned_state << ", exit program." << std::endl;
    return EXIT_FAILURE;
  }

  sl::RuntimeParameters runtime_parameters;
  runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD; // Use STANDARD sensing mode

  sl::Mat left_sl, depth_image, point_cloud;
  cv::Mat left_cv_bgra, left_cv_bgr;

  QR_code_detection qr;

  while(rclcpp::ok())
  {
    if (zed.grab() == sl::ERROR_CODE::SUCCESS) 
    {
      zed.retrieveImage(left_sl, sl::VIEW::LEFT);
      zed.retrieveImage(depth_image, sl::VIEW::DEPTH);
      zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA);

      // Preparing inference
      left_cv_bgra = qr.slMat2cvMat(left_sl);
      cv::cvtColor(left_cv_bgra, left_cv_bgr, cv::COLOR_BGRA2BGR);

      if (left_cv_bgr.empty()) continue;

      // Variable for decoded objects
      vector<decodedObject> decodedObjects;
      qr.decode(left_cv_bgr, decodedObjects);
      qr.display(left_cv_bgr, decodedObjects, point_cloud, left_sl);

      // cv::imshow("Results", left_cv_bgr);

      // char c=(char)cv::waitKey(25);
      // if(c==27)
      // {
      //     break;
      // }
    }
  }
  rclcpp::spin(std::make_shared<QR_code_detection>());
  zed.close();
  // cv::destroyAllWindows();s
  rclcpp::shutdown();
  return EXIT_SUCCESS;
//   return 0;
}