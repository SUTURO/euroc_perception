#include "suturo_perception_segmentation/task4_segmenter.h"

#include <perception_utils/point_cloud_operations.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
#include <cmath>

#define PI 3.14159265

using namespace suturo_perception;


Task4Segmenter::Task4Segmenter(ros::NodeHandle &node, bool isTcp, suturo_msgs::Task task) 
: Segmenter(), nodeHandle_(node), isTcp_(isTcp), task_(task)
{
	logger = Logger("task4_segmenter");
	
	updateSegmentationCloud();
}

void Task4Segmenter::updateSegmentationCloud()
{
	logger.logInfo("generating segmentation cloud");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr odom_segmentation_cloud = generate_simple_segmentation_cloud();
	segmentation_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	logger.logInfo("transforming segmentation cloud");
	sensor_msgs::PointCloud2 odom_pc2;
  pcl::toROSMsg(*odom_segmentation_cloud, odom_pc2 );
  odom_pc2.header.frame_id = "/odom_combined";
  odom_pc2.header.stamp = ros::Time(0);
	
	sensor_msgs::PointCloud2 depth_pcl_pc2;
	
	//logger.logInfo("Waiting for tf to come up...");
	//sleep(6);
	tf::TransformListener listener;
	tf::StampedTransform transform;
  int tries = 0;
  ros::Rate rate(10.0);
	transform_success_ = false;
  while (nodeHandle_.ok() && tries < 10 && !transform_success_)
  {
    try
    {
			std::string frame_from = "/odom_combined";
			std::string frame_to = "/tdepth_pcl";
			if (!isTcp_)
				frame_to = "/sdepth_pcl";
      listener.waitForTransform(frame_from, frame_to, ros::Time(0), ros::Duration(6.0));
      listener.lookupTransform(frame_from, frame_to, ros::Time(0), transform);
      pcl_ros::transformPointCloud(frame_to, odom_pc2, depth_pcl_pc2, listener);
      transform_success_ = true;
    }
    catch (tf::TransformException &ex) 
    {
      logger.logError((boost::format("TransformException\n%s") % ex.what()).str());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
    tries++;
		logger.logWarn((boost::format("transform attempt %s") % tries).str());
  }
  
  if (!transform_success_)
	{
		logger.logError("couldn't transform! segmentation won't work!");
	}
	else
	{
		pcl::fromROSMsg(depth_pcl_pc2,*segmentation_cloud_);
	}
}

bool Task4Segmenter::clusterPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<pcl::PointIndices::Ptr> &original_indices, PipelineData::Ptr &pipeline_data)
{

  if(object_clusters->points.size() < 50)
  {
    logger.logError("clusterPointcloud: original_cloud has less than 50 points. Skipping ...");
    return false;
  }

  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  // Identify clusters in the input cloud
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (object_clusters);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (pipeline_data->ecObjClusterTolerance);
  ec.setMinClusterSize (pipeline_data->ecObjMinClusterSize);
  ec.setMaxClusterSize (pipeline_data->ecObjMaxClusterSize);
  ec.setSearchMethod (tree);
  ec.setInputCloud (object_clusters);
  ec.extract(cluster_indices);
  logger.logInfo((boost::format("Found %s clusters.") % cluster_indices.size()).str());

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "filtering out objects above the plane");

  int i=0;
  // Iterate over the found clusters and extract single pointclouds
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    if (it->indices.size() < 10)
    {
      logger.logError("Cloud cluster has less than 10 points, skipping...");
      continue;
    }
    // Gather all points for a cluster into a single pointcloud
    boost::posix_time::ptime s1 = boost::posix_time::microsec_clock::local_time();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (object_clusters->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    i++;
    extracted_objects.push_back(cloud_cluster);
     // TODO fill in object indices
  }
  return true;

  boost::posix_time::ptime e2 = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e2, "clusterPointcloud");

}
bool 
Task4Segmenter::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
    PipelineData::Ptr &pipeline_data, 
    PipelineObject::VecPtr &pipeline_objects)
{
	if (!transform_success_)
	{
		logger.logError("transform of segmentation cloud failed, segmentation won't work. Please restart the node");
		return false;
	}
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  // Build a filter to filter on the Z Axis
  pcl::PassThrough<pcl::PointXYZRGB> pass(true);
  PointCloudOperations::filterZAxis(cloud_in, cloud_filtered, pass, pipeline_data->zAxisFilterMin, pipeline_data->zAxisFilterMax);
  logger.logInfo((boost::format("PointCloud: %s data points") % cloud_in->points.size()).str());

  std::vector<int> removed_indices_filtered;
  removed_indices_filtered = *pass.getIndices();
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "z-filter");
  
  logger.logInfo((boost::format("PointCloud after z-filter: %s data points") % cloud_filtered->points.size()).str());

  //voxelizing cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
  PointCloudOperations::downsample(cloud_filtered, cloud_downsampled, pipeline_data->downsampleLeafSize);
  cloud_filtered = cloud_downsampled; // Use the downsampled cloud now
  
  logger.logInfo((boost::format("PointCloud after downsample: %s data points") % cloud_filtered->points.size()).str());
  downsampled_pointcloud_ = cloud_filtered;
  
  logger.logInfo((boost::format("segmentation_cloud_ has %s points") % segmentation_cloud_->points.size()).str());

  // Find the biggest table plane in the scene
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  PointCloudOperations::fitPlanarModel(segmentation_cloud_, inliers, coefficients, pipeline_data->planeMaxIterations, pipeline_data->planeDistanceThreshold);
  logger.logInfo((boost::format("Table inlier count: %s") % inliers->indices.size ()).str());
  logger.logInfo((boost::format("pcl::ModelCoefficients: %s") % coefficients->values.size()).str());
  for (int i = 0; i < coefficients->values.size(); i++)
  {
    logger.logInfo((boost::format("  %s") % coefficients->values[i]).str());
  }
  pipeline_data->coefficients_ = coefficients;
	
	// Extract all objects above
	// the table plane
	pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());

	// Remove all NaNs from the PointCloud. Otherwise, we can't use Euclidean Clustering (KDTree)
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr nanles_cloud (new pcl::PointCloud<pcl::PointXYZRGB>()); // NEW
	PointCloudOperations::removeNans(cloud_in, nanles_cloud); // NEW

	PointCloudOperations::extractAllPointsAbovePointCloud(nanles_cloud, segmentation_cloud_, // New
			object_clusters, object_indices, 2, pipeline_data->prismZMin, pipeline_data->prismZMax);
	logger.logInfo((boost::format("After extractAllPointsAbovePointCloud: %s indices and %s object_cluster pts") % object_indices->indices.size() % object_clusters->points.size() ).str() );
	points_above_table_ = object_clusters;
	
	e = boost::posix_time::microsec_clock::local_time();
	logger.logTime(s, e, "extractAllPointsAbovePointCloud");

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects;
	std::vector<pcl::PointIndices::Ptr> extractedIndices;
	clusterPointcloud(object_clusters, extractedObjects, extractedIndices, pipeline_data); // NEW - Just cluster everything above the table - This is unfortunately slower then the projection method ...
	logger.logInfo((boost::format(" - extractedObjects Vector size %s") % extractedObjects.size()).str());
	logger.logInfo((boost::format(" - extractedIndices  Vector size %s") % extractedIndices.size()).str());

	e = boost::posix_time::microsec_clock::local_time();
	logger.logTime(s, e, "table from pointcloud");

	// publish the segmentation results
	pipeline_objects.clear();
	for (int i = 0; i < extractedObjects.size(); i++)
	{
		PipelineObject::Ptr pipelineObject(new PipelineObject);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr it = extractedObjects.at(i);
		logger.logInfo((boost::format("Cluster %i has %s points") % i % it->points.size()).str());

		if(it->points.size()<50)
		{
			logger.logError("Cluster cloud has less than 50 points. Skipping ...");
			continue;
		}

		pipelineObject->set_pointCloud(it);
		pipeline_objects.push_back(pipelineObject);
	}

	table_pointcloud_ = segmentation_cloud_;
	
	e = boost::posix_time::microsec_clock::local_time();
	logger.logTime(s, e, "full segmentation");

  return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Task4Segmenter::generate_simple_segmentation_cloud()
{

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentation_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	segmentation_cloud->height = 1;
	segmentation_cloud->width = 4;
	segmentation_cloud->is_dense = false;
	segmentation_cloud->points.resize(4);

	segmentation_cloud->points[0].x = -1.0;
	segmentation_cloud->points[0].y = -1.0;
	segmentation_cloud->points[0].z = 0.0;
	
	segmentation_cloud->points[1].x = 1.0;
	segmentation_cloud->points[1].y = -1.0;
	segmentation_cloud->points[1].z = 0.0;
	
	segmentation_cloud->points[2].x = -1.0;
	segmentation_cloud->points[2].y = 1.0;
	segmentation_cloud->points[2].z = 0.0;
	
	segmentation_cloud->points[3].x = 1.0;
	segmentation_cloud->points[3].y = 1.0;
	segmentation_cloud->points[3].z = 0.0;
	
	return segmentation_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Task4Segmenter::getTablePointCloud()
{
  return table_pointcloud_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Task4Segmenter::getDownsampledPointCloud()
{
  return downsampled_pointcloud_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Task4Segmenter::getPointsAboveTable()
{
  return points_above_table_;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Task4Segmenter::getProjectedPoints()
{
  return projected_points_;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Task4Segmenter::getProjectionClusters()
{
  return projection_clusters_;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Task4Segmenter::getProjectionClusterHulls()
{
  return projection_cluster_hulls_;
}
