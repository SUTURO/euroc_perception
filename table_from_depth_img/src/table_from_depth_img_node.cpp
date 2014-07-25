#include "table_from_depth_img_node.h"

#include "perception_utils/point_cloud_operations.h"

#include <pcl/filters/passthrough.h>

using namespace perception_utils;

TableFromDepthImageNode::TableFromDepthImageNode(ros::NodeHandle &n, std::string depthTopic) : 
  nodeHandle_(n), 
  cloudTopic_(depthTopic)
{
	logger = perception_utils::Logger("TableFromDepthImageNode");
  clusterService = nodeHandle_.advertiseService("/suturo/GetTable", 
    &TableFromDepthImageNode::getTable, this);
}

bool
TableFromDepthImageNode::getTable(suturo_perception_msgs::GetTable::Request &req, suturo_perception_msgs::GetTable::Response &res)
{

  return true;
}

void
TableFromDepthImageNode::receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*inputCloud,*cloud_in);
  
  // Set default parameters
  float zAxisFilterMin = 0.0;
  float zAxisFilterMax = 1.5;
  float downsampleLeafSize = 0.01;
  int planeMaxIterations = 1000;
  double planeDistanceThreshold = 0.01;
  double ecClusterTolerance = 0.02; // 2cm
  int ecMinClusterSize = 6000;
  int ecMaxClusterSize = 200000;  
  double prismZMin = 0.02;
  double prismZMax = 0.50; // cutoff 50 cm above plane
  double ecObjClusterTolerance = 0.03; // 3cm
  int ecObjMinClusterSize = 100;
  int ecObjMaxClusterSize = 25000;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      objects_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>);

	// Build a filter to filter on the Z Axis
  pcl::PassThrough<pcl::PointXYZRGB> pass(true);
  PointCloudOperations::filterZAxis(cloud_in, cloud_filtered, pass, zAxisFilterMin, zAxisFilterMax);
  logger.logInfo((boost::format("PointCloud: %s data points") % cloud_filtered->points.size()).str());

  std::vector<int> removed_indices_filtered;
  removed_indices_filtered = *pass.getIndices();
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "z-filter");

  //voxelizing cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
  PointCloudOperations::downsample(cloud_filtered, cloud_downsampled, downsampleLeafSize);
  cloud_filtered = cloud_downsampled; // Use the downsampled cloud now

  // Find the biggest table plane in the scene
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  PointCloudOperations::fitPlanarModel(cloud_filtered, inliers, coefficients, planeMaxIterations, planeDistanceThreshold);
  logger.logInfo((boost::format("Table inlier count: %s") % inliers->indices.size ()).str());
  
  // Extract the plane as a PointCloud from the calculated inliers
  PointCloudOperations::extractInliersFromPointCloud(cloud_filtered, inliers, cloud_plane, false);

  // Take the biggest cluster in the extracted plane. This will be
  // most likely our desired table pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices);
  PointCloudOperations::extractBiggestCluster(cloud_plane, plane_cluster, inliers, new_inliers,
    ecObjClusterTolerance, ecMinClusterSize, ecMaxClusterSize);

  // NOTE: We need to transform the inliers from table_cluster_indices to inliers
  inliers = new_inliers;
  
  if(inliers->indices.size () == 0)
  {
    logger.logError("Second Table Inlier Set is empty. Exiting....");
    return;
  }

  e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "table from pointcloud");
}
