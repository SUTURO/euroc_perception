#include "suturo_perception_segmentation/task4_segmenter.h"

#include <perception_utils/point_cloud_operations.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/planar_polygon_fusion.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/plane_coefficient_comparator.h>
#include <pcl/segmentation/euclidean_plane_coefficient_comparator.h>
#include <pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>

#define PI 3.14159265

using namespace suturo_perception;


Task4Segmenter::Task4Segmenter(ros::NodeHandle &node, bool isTcp, suturo_msgs::Task task) 
: Segmenter(), nodeHandle_(node), isTcp_(isTcp), task_(task)
{
	logger = Logger("task4_segmenter");
	
	table_pointcloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	downsampled_pointcloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	points_above_table_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	projected_points_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	updateSegmentationCloud(PipelineData::Ptr(new PipelineData()));
}

void Task4Segmenter::updateSegmentationCloud(PipelineData::Ptr pipeline_data)
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
		return;
	}

	pcl::fromROSMsg(depth_pcl_pc2,*segmentation_cloud_);

	// fit the table plane to get coefficients
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  PointCloudOperations::fitPlanarModel(segmentation_cloud_, inliers, coefficients, pipeline_data->planeMaxIterations, pipeline_data->planeDistanceThreshold);
  logger.logInfo((boost::format("Table inlier count: %s") % inliers->indices.size ()).str());
  logger.logInfo((boost::format("pcl::ModelCoefficients: %s") % coefficients->values.size()).str());
  for (int i = 0; i < coefficients->values.size(); i++)
  {
    logger.logInfo((boost::format("  %s") % coefficients->values[i]).str());
  }
  table_coefficients_ = coefficients;
}

bool 
Task4Segmenter::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
    PipelineData::Ptr &pipeline_data, 
    PipelineObject::VecPtr &pipeline_objects)
{
	if (!transform_success_)
	{
		logger.logError("Initialization failed! call updateSegmentationCloud! Can't segment");
		return false;
	}
  boost::posix_time::ptime mps_start = boost::posix_time::microsec_clock::local_time();
	
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	
	// self made plane generation
	double max_plane_dist = 0.005; // 5mm
	
	labels->points.resize(cloud->points.size());
	// init height calculation
  float a,b,c,d,e;
  a = table_coefficients_->values[0];
  b = table_coefficients_->values[1];
  c = table_coefficients_->values[2];
  d = table_coefficients_->values[3];
  e = sqrt(a*a + b*b + c*c);
	
#pragma omp parallel for
	for (int i = 0; i < cloud->points.size(); i++)
	{
		pcl::PointXYZRGB *p = &cloud->points[i];
		// height calculation
		double tmp = ( a * p->x + b * p->y + c * p->z + d ) / e;
    tmp = tmp < 0 ? -tmp : tmp; // abs
    
    labels->points[i].label = tmp < max_plane_dist ? 0 : 1;
	}
	
	//Segment Objects
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clusters;
	std::vector<bool> plane_labels;
	plane_labels.push_back(true);
	plane_labels.push_back(false);
	
	pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr  euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label> ());
	euclidean_cluster_comparator_->setInputCloud (cloud);
	euclidean_cluster_comparator_->setLabels (labels);
	euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
	euclidean_cluster_comparator_->setDistanceThreshold (pipeline_data->ecObjClusterTolerance, false);
	logger.logInfo((boost::format("euclidean_cluster_comparator->distanceThreshold = %s") % pipeline_data->ecObjClusterTolerance).str());
	
	pcl::PointCloud<pcl::Label> euclidean_labels;
	std::vector<pcl::PointIndices> euclidean_label_indices;
	pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGB,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
	euclidean_segmentation.setInputCloud (cloud);
	euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
	
	for (size_t i = 0; i < euclidean_label_indices.size (); i++)
	{
		if (euclidean_label_indices[i].indices.size () > pipeline_data->ecObjMinClusterSize)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,*cluster);
			clusters.push_back (cluster);
			logger.logInfo((boost::format("euclidean cluster %s has %s points!") % i % cluster->points.size()).str());
		}
	}
	logger.logInfo((boost::format("Got %s euclidean clusters!") % clusters.size() ).str());
	projection_clusters_ = clusters;

	// publish the segmentation results
	pipeline_objects.clear();
	for (int i = 0; i < clusters.size(); i++)
	{
		PipelineObject::Ptr pipelineObject(new PipelineObject);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr it = clusters.at(i);
		logger.logInfo((boost::format("Cluster %i has %s points") % i % it->points.size()).str());

		if(it->points.size()<50)
		{
			logger.logError("Cluster cloud has less than 50 points. Skipping ...");
			continue;
		}

		pipelineObject->set_pointCloud(it);
		pipeline_objects.push_back(pipelineObject);
	}

	// take the fake table plane cloude & coefficients
	table_pointcloud_ = segmentation_cloud_;
  pipeline_data->coefficients_ = table_coefficients_;
	
  boost::posix_time::ptime mps_end = boost::posix_time::microsec_clock::local_time();
	logger.logTime(mps_start, mps_end, "Segmentation took: ");
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
