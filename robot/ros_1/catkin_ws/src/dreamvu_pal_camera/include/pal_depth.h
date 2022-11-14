#ifndef PAL_DEPTH
#define PAL_DEPTH

#include"ros/ros.h"
#include <image_transport/image_transport.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <sstream> 
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/make_shared.hpp>
#include "PAL.h"
#include "dreamvu_pal_camera/BoundingBox.h"
#include "dreamvu_pal_camera/BoundingBoxArray.h"
#include <fcntl.h>
#include <unistd.h>

using namespace std;
using namespace cv_bridge;

class PersonDepth
{
    public:
        PersonDepth(ros::NodeHandle &n);
        //PersonDepth();
        cv_bridge::CvImagePtr cv_ptr;
        int depth_size;
        int mid_point_x, mid_point_y;
        float mid_x, mid_y;
        vector<float> depth_pos;
        float depth_min;
        int depth_val;
	void getDepth(const sensor_msgs::Image::ConstPtr& msg);
	void getBox(const dreamvu_pal_camera::BoundingBoxArray::ConstPtr &cord);


    private:
        //ros::NodeHandle n;
        image_transport::ImageTransport it;

        image_transport::Subscriber depth_image;
        //ros::Subscriber depth_image;
        ros::Subscriber bounding_box;

	ros::Publisher afs_state;

	std_msgs::Int32 state;
};
#endif
//http://wiki.ros.org/image_transport/Tutorials

/*
If you use image transport change the constructor definition,  and initialize the image transport and subscriber.
Also, the class definition in the cpp file changes.
*/
