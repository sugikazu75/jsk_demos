/**
 * Makes all the non-planar things green, publishes the final result on the /obstacles topic.
 */
#include <iostream>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>

ros::Publisher pub;

void cloud_cb (const pcl::PCLPointCloud2ConstPtr& cloud_blob)
{
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.02);
  seg.setInputCloud (cloud_filtered);

  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  pcl::IndicesPtr remaining (new std::vector<int>);
  remaining->resize (nr_points);
  for (size_t i = 0; i < remaining->size (); ++i) { (*remaining)[i] = static_cast<int>(i); }

  // While 30% of the original cloud is still there
  while (remaining->size () > 0.5 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setIndices (remaining);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0) break;

    // Extract the inliers
    std::vector<int>::iterator it = remaining->begin();
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
      int curr = inliers->indices[i];
      // Remove it from further consideration.
      while (it != remaining->end() && *it < curr) { ++it; }
      if (it == remaining->end()) break;
      if (*it == curr) it = remaining->erase(it);
    }
    i++;
  }
  std::cout << "Found " << i << " planes." << std::endl;
  // std::cout << inliers->indices.at(i) << std::endl;
  // std::cout << inliers->indices.size() << std::endl;
  if(i > 0){
    std::cout << coefficients -> values[0] << "x + ";
    std::cout << coefficients -> values[1] << "y + ";
    std::cout << coefficients -> values[2] << "z + ";
    std::cout << coefficients -> values[3] << "= 0" << std::endl;;
    std::cout << "distance to plane = " << std::abs(coefficients->values[3]) / std::sqrt(coefficients->values[0]*coefficients->values[0] + coefficients->values[1]*coefficients->values[1] + coefficients->values[2]*coefficients->values[2]) << std::endl;
  }

  // Color all the non-planar things.
  for (std::vector<int>::iterator it = remaining->begin(); it != remaining->end(); ++it)
  {
    uint8_t r = 0, g = 255, b = 0;
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    cloud_filtered->at(*it).rgb = *reinterpret_cast<float*>(&rgb);
  }

  // Publish the planes we found.
  pcl::PCLPointCloud2 outcloud;
  pcl::toPCLPointCloud2 (*cloud_filtered, outcloud);
  pub.publish (outcloud);

  // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  // viewer.showCloud(cloud_filtered->makeShared());
  // while(!viewer.wasStopped()){
  // }

}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "obstacles");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/remote/stereo/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PCLPointCloud2> ("obstacles", 1);

  // Spin
  ros::spin ();
}
