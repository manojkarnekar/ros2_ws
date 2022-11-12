#include "processPointClouds.h"
#include "processPointClouds.cpp"
// #include <jb_pcl/ClusterData.h>
// template<typename PointT>
ros::Publisher pub;
ProcessPointClouds<pcl::PointXYZRGB>* point_processor = new ProcessPointClouds<pcl::PointXYZRGB>();

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Convert to PCL PointCloud<PointXYZRGB> since the SAC segmentation API does not work with PCL::PCLPointCloud2
  // and we dont want to loose the color information.
  pcl::fromROSMsg(*cloud_msg, *cloud);

  Eigen::Vector4f minPoint (-30, -6.5, -3, 1);
  Eigen::Vector4f maxPoint (30, 6.5, 10, 1);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterCloud = point_processor->FilterCloud(cloud, 0.001, minPoint, maxPoint);

  std::cout << "org size: " << cloud_msg->width * cloud_msg->height << endl;
  std::cout << "filtered size: " << filterCloud->width * filterCloud->height << endl;
  
  std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> seg_result_pair = point_processor->SegmentPlane (filterCloud, 100, 0.2);

  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters_cloud = point_processor->Clustering(seg_result_pair.first, 0.02, 2, 5);

  for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster: clusters_cloud)
  {
    Box box = point_processor->BoundingBox(cluster);
    
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cluster, output);
    // Publish the data
    pub.publish (output);

  }

  
  ROS_DEBUG("Publishing segmented cloud");

  // Convert to ROS data type
  

}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_segmentaion_node");
  ros::NodeHandle nh;
  ROS_INFO("Started pcl_segmentaion node");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/zed2/zed_node/point_cloud/cloud_registered", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("cloud/filtered", 1);

  // Spin
  ros::spin ();
}