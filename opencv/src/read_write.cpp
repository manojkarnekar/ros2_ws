//Include Libraries
#include<opencv2/opencv.hpp>
#include<iostream>
 
// Namespace nullifies the use of cv::function(); 
using namespace std;
using namespace cv;
 
// Read an image 

int main(int argc, char** argv)
{
    cv::Mat img_grayscale = imread("/home/manoj/ros_ws2/opencv/src/lion.jpeg", 0);
    
    // Display the image.
    imshow("grayscale image", img_grayscale); 
    
    // Wait for a keystroke.   
    waitKey(0);  
    
    // Destroys all the windows created                         
    destroyAllWindows();
    
    // Write the image in the same directory
    imwrite("/home/manoj/ros_ws2/opencv/grayscale.jpg", img_grayscale);

    return 0;
}