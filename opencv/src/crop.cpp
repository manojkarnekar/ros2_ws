#include<opencv2/opencv.hpp>
#include<iostream>
 
// Namespace nullifies the use of cv::function();
using namespace std;
using namespace cv;
 
int main()
{
  // Read image
  Mat img = imread("/home/manoj/ros_ws2/opencv/src/tree.jpg");
  cout << "Width : " << img.size().width << endl;
  cout << "Height: " << img.size().height << endl;
  cout<<"Channels: :"<< img.channels() << endl;
  // Crop image
  Mat cropped_image = img(Range(80,280), Range(150,330));
 
  //display image
  imshow(" Original Image", img);
  imshow("Cropped Image", cropped_image);
 
  //Save the cropped Image
  imwrite("/home/manoj/ros_ws2/opencv/src/tree_crop.jpg", cropped_image);
 
  // 0 means loop infinitely
  waitKey(0);
  destroyAllWindows();
  return 0;
}