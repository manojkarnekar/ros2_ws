#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;
 
int main( int argc, char** argv )
{
 
  // Read image 
  Mat src = imread("/home/manoj/ros_ws2/opencv/src/threshold.jpg", IMREAD_GRAYSCALE); 
  Mat dst; 
   
  // Basic threhold example 
  threshold(src,dst,0, 255, THRESH_BINARY); 
  imwrite("/home/manoj/ros_ws2/opencv/images/opencv-threshold-example.jpg", dst); 
 
  // Thresholding with maxval set to 128
  threshold(src, dst, 0, 128, THRESH_BINARY); 
  imwrite("/home/manoj/ros_ws2/opencv/images/opencv-thresh-binary-maxval.jpg", dst); 
   
  // Thresholding with threshold value set 127 
  threshold(src,dst,127,255, THRESH_BINARY); 
  imwrite("/home/manoj/ros_ws2/opencv/images/opencv-thresh-binary.jpg", dst); 
   
  // Thresholding using THRESH_BINARY_INV 
  threshold(src,dst,127,255, THRESH_BINARY_INV); 
  imwrite("/home/manoj/ros_ws2/opencv/images/opencv-thresh-binary-inv.jpg", dst); 
   
  // Thresholding using THRESH_TRUNC 
  threshold(src,dst,127,255, THRESH_TRUNC); 
  imwrite("/home/manoj/ros_ws2/opencv/images/opencv-thresh-trunc.jpg", dst); 
 
  // Thresholding using THRESH_TOZERO 
  threshold(src,dst,127,255, THRESH_TOZERO); 
  imwrite("/home/manoj/ros_ws2/opencv/images/opencv-thresh-tozero.jpg", dst); 
 
  // Thresholding using THRESH_TOZERO_INV 
  threshold(src,dst,127,255, THRESH_TOZERO_INV); 
  imwrite("/home/manoj/ros_ws2/opencv/images/opencv-thresh-to-zero-inv.jpg", dst); 
} 