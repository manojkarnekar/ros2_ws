#include "pal_depth.h"

PersonDepth::PersonDepth()
{
    depth_image = n.subscribe("/dreamvu/pal/persons/get/depth", 10, &PersonDepth::getDepth, this);
    bounding_box = n.subscribe("/dreamvu/pal/persons/get/detection_boxes", 10, &PersonDepth::getBox, this);
    afs_state = n.advertise<std_msgs::Int32>("/dreamvu/pal/persons/get/afs_state", 10);
}

void PersonDepth::getDepth(const sensor_msgs::Image::ConstPtr& msg)
{
      depth_size = msg->data.size();

      try
      {
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
      }
      catch(cv_bridge::Exception& e)
      {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
      }

      //cv::Mat &mat = cv_ptr->image;
      //cv_ptr->image.cols
      //ROS_INFO("Mat col size: %d", mat.cols);
      //ROS_INFO("Mat row size: %d", mat.rows);
      //ROS_INFO("Point print: %f", mat.at<float>(500, 500));
}

void PersonDepth::getBox(const dreamvu_pal_camera::BoundingBoxArray::ConstPtr& cord)
{   
    if(cv_ptr != NULL)
    {
    //ROS_INFO("Mat col size: %d", cv_ptr->image.cols);
    cv::Mat &mat = cv_ptr->image;
    for(int i=0; i < cord->boxes.size(); i++)
    {
        const dreamvu_pal_camera::BoundingBox &points = cord->boxes[i];
        
        mid_point_x = points.x1 + points.x2;
	mid_point_y = points.y1 + points.y2;

	mid_x = mid_point_x / 2;
        mid_y = mid_point_y / 2;
  
	depth_pos = mat.at<float>(mid_y, mid_x);
        ROS_INFO("Depth Pos %f", depth_pos);


        if(depth_pos < 200)
        {
            state.data = 1;
	}

        else
        {
            state.data = 2; //continue
        }

        afs_state.publish(state);
    }
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "afs_person_depth");
        //ros::NodeHandle n;

	PersonDepth pal;	

	
	ros::spin();
	return 0;
}

