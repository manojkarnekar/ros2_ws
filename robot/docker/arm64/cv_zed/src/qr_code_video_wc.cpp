#include <iostream>
#include <algorithm>
#include <vector>
#include <zbar.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;
using namespace zbar;

typedef struct
{
  string type;
  string data;
  vector <Point> location;
}decodedObject;

// Find and decode barcodes and QR codes
void decode(Mat &im, vector<decodedObject>&decodedObjects)
{
  ImageScanner scanner;
  scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 0);
  scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
  Mat imGray;
  cvtColor(im, imGray, COLOR_BGR2GRAY);
  GaussianBlur(imGray, imGray, Size(5, 5), 0);
  // GaussianBlur(im, (5, 5), 0);
  Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

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
    // Obtain location
    for(int i = 0; i< symbol->get_location_size(); i++)
    {
      obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
    }

    decodedObjects.push_back(obj);
  }
}

// Display barcode and QR code location
void display(Mat &im, vector<decodedObject>&decodedObjects)
{
  // Loop over all decoded objects
  for(int i = 0; i < decodedObjects.size(); i++)
  {
    vector<Point> points = decodedObjects[i].location;
    vector<Point> hull;

    // If the points do not form a quad, find convex hull
    if(points.size() > 4)
      convexHull(points, hull);
    else
      hull = points;
    

    // Number of points in the convex hull
    int n = hull.size();

    for(int j = 0; j < n; j++)
    {
      line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
      cout<<hull[j]<<" ";
    }
    cout<<endl;

    circle(im, Point((hull[0], hull[1])), 5, Scalar(0,255,0),-1, 1,0);
    circle(im, Point((hull[1], hull[0])), 5, Scalar(0,255,0),-1, 1,0);
    circle(im, Point((hull[2], hull[3])), 5, Scalar(0,255,0),-1, 1,0);
    circle(im, Point((hull[3], hull[2])), 5, Scalar(0,255,0),-1, 1,0);

    // left mid point
    circle(im, Point(((hull[0]+hull[1])/2.0, (hull[1]+hull[0])/2.0)), 5, Scalar(0,255,0),-1, 1,0);

    // right mid point
    circle(im, Point(((hull[2]+hull[3])/2.0, (hull[3]+hull[2])/2.0)), 5, Scalar(0,255,0),-1, 1,0);

    //mid point
    Point pt = ((hull[0]+hull[1]+hull[2]+hull[3])/4.0 , (hull[1]+hull[0] + hull[3]+hull[2])/4.0);
    circle(im, Point(pt.x , pt.y), 5, Scalar(0,255,0),-1, 1,0);

    for(long int i=0; i<=15; i++)
    {
      circle(im, Point(pt.x+i , pt.y+i), 2, Scalar(0,255,0),-1, 1,0);
      circle(im, Point(pt.x+i , pt.y-i), 2, Scalar(0,255,0),-1, 1,0);
      circle(im, Point(pt.x-i , pt.y+i), 2, Scalar(0,255,0),-1, 1,0);
      circle(im, Point(pt.x-i , pt.y-i), 2, Scalar(0,255,0),-1, 1,0);
    }
    
    cout<<pt.x<<" "<<pt.y<<endl;

    


  }

  // Display results
  imshow("Results", im);

  

}

int main(int argc, char *argv[])
{
  // Read image
  const char* imagepath = argv[1];
  VideoCapture cap(0); 
   
  // Check if camera opened successfully
  if(!cap.isOpened())
  {
    cout << "Error opening video stream or file" << endl;
    return -1;
  }

  while(1)
  {
    Mat frame;
    cap >> frame;
    if (frame.empty())
    {
        break;
    }

    // Variable for decoded objects
    vector<decodedObject> decodedObjects;

    // Find and decode barcodes and QR codes
    decode(frame, decodedObjects);
    display(frame, decodedObjects);

    char c=(char)waitKey(25);
    if(c==27)
    {
        break;
    }
  }
  
  cap.release();
  destroyAllWindows();


  return 0;
 }