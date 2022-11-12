#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <jb_pcl/SegmentedClustersArray.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "box.h"
#include <unordered_set>
#include "processPointClouds.h"
// #include <jb_pcl/ClusterData.h>
// template<typename PointT>
ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr (cloud);

  // Container for filtered data
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.05, 0.05, 0.05);
  sor.filter (*cloudFilteredPtr);

  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud);

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);
  //perform passthrough filtering

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the passrhrough filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (.5, 1.1);
  pass.filter (*xyzCloudPtrFiltered);

  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);

  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  seg1.setOptimizeCoefficients (true);
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.04);
  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);

  // perform euclidean cluster segmentation to seporate individual objects
  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);
  
  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract(cluster_indices);

  // declare the output variable instances
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 outputPCL;

  // declare an instance of the SegmentedClustersArray message
  jb_pcl::SegmentedClustersArray CloudClusters;

  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
     // create a pcl object to hold the extracted cluster
      pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

      // now we are in a vector of indices pertaining to a single cluster.
      // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      {
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);
      }
      pcl::toPCLPointCloud2(*xyzCloudPtrRansacFiltered ,outputPCL);
      pcl_conversions::fromPCL(outputPCL, output);
      CloudClusters.clusters.push_back(output);
  }

  pub.publish(CloudClusters);

//   pcl::toPCLPointCloud2(*xyzCloudPtrRansacFiltered ,outputPCL);
//   pcl_conversions::fromPCL(outputPCL, output);

//   pub.publish (output);




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