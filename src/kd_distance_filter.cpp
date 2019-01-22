#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

ros::Publisher pub;

void chatterCallback(const sensor_msgs::PointCloud2ConstPtr& input_cloud) 
{
  //create containers and do conversions
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                         cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  //pcl::PointCloud2 cloud_filtered;
  
  pcl::fromROSMsg(*input_cloud, *cloud);
  
  //set up the tree
  pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
  kdtree.setInputCloud (cloud);
  pcl::PointXYZRGB searchPoint;
  
  //decide from where we search - I think it makes sense to search from 0,0,0
  searchPoint.x = 0.0;
  searchPoint.y = 0.0;
  searchPoint.z = 0.0;
  
  //define a radius
  double radius = 1.0;
  
  //prepare for the search
  std::vector<int> pointIndicesOut;
  std::vector<float> pointRadiusSquaredDistance;
  
  kdtree.radiusSearch(searchPoint, radius, pointIndicesOut, pointRadiusSquaredDistance);
  
  for(size_t i = 0; i < pointIndicesOut.size(); ++i)
  {
    cloud_filtered->push_back(cloud->points[ pointIndicesOut[i] ]);
  }
  
  //return back to ROS message
  sensor_msgs::PointCloud2 output_cloud;
  pcl::toROSMsg(*cloud_filtered, output_cloud);
  output_cloud.header.frame_id="camera_depth_optical_frame";
  output_cloud.header.stamp=ros::Time::now();
  
  pub.publish(output_cloud);
}

int main(int argc, char **argv)
{
  //initialize a ros node
  ros::init(argc, argv, "distance_filter");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  //subscribe to 
  ros::Subscriber sub = n.subscribe("/camera/depth/color/points", 1, chatterCallback);
                  pub = n.advertise<sensor_msgs::PointCloud2>("camera/depth/color/filtered_points", 1);

  ros::spin();

  return 0;
}