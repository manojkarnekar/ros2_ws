#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;
int main()
{
    // Read Images
    Mat img = imread("/home/manoj/ros_ws2/opencv/src/tree.jpg");
    // Display Image
    imshow("Original Image", img);
    waitKey();
    // Print Error message if image is null
    if (img.empty())
        {
            cout << "Could not read image" << endl;
        }
    // Draw line on image
    Mat imageLine = img.clone();
    Mat imageCircle = img.clone();
    Mat rect = img.clone();
    Point pointA(200,80);
    Point pointB(450,80);
    Point pointC(450, 200);

    line(imageLine, pointA, pointB, Scalar(255, 255, 0), 3, 4, 0);

    circle(imageCircle, pointA, 20, Scalar(255, 0, 0), -1, 4, 0);

    rectangle(rect, pointA, pointC, Scalar(0, 255, 255), 3, 8, 0);
    imshow("Lined Image", imageLine);
    imshow("circled Image", imageCircle);
    imshow("rect", rect);
    waitKey();
}