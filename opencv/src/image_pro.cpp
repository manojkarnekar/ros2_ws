#include<opencv2/opencv.hpp>
#include<iostream>
 
// Namespace to nullify use of cv::function(); syntax
using namespace std;
using namespace cv;
 
int main()
{
  // Read the image using imread function
  Mat image = imread("/home/manoj/ros_ws2/opencv/src/tree.jpg");
  imshow("Original Image", image);
 
 
  cout << "height = " << image.size().height << endl;
  cout << "height = " << image.size().width << endl;
  // let's downscale the image using new  width and height
  int down_width = 300;
  int down_height = 200;
  Mat resized_down;
  //resize down
  resize(image, resized_down, Size(down_width, down_height), INTER_LINEAR);
  // let's upscale the image using new  width and height
  int up_width = 600;
  int up_height = 400;
  Mat resized_up;
  //resize up
  resize(image, resized_up, Size(up_width, up_height), INTER_LINEAR);
  // Display Images and press any key to continue
  imshow("Resized Down by defining height and width", resized_down);
  waitKey();
  imshow("Resized Up image by defining height and width", resized_up);
  waitKey();
 
 
  destroyAllWindows();
  return 0;
}