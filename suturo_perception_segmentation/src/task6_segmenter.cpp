#include "suturo_perception_segmentation/task6_segmenter.h"

#include <perception_utils/point_cloud_operations.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
#include <cmath>

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


Task6Segmenter::Task6Segmenter(ros::NodeHandle &node, bool isTcp, suturo_msgs::Task task) 
: Segmenter(), nodeHandle_(node), isTcp_(isTcp), task_(task)
{
	logger = Logger("task6_segmenter");
	
	updateConveyorCloud(PipelineData::Ptr(new PipelineData()));
}

void Task6Segmenter::updateConveyorCloud(PipelineData::Ptr pipeline_data)
{
	logger.logInfo("generating conveyor cloud");
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr odom_conveyor_cloud = generate_simple_conveyor_cloud();
	conveyor_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	
	logger.logInfo("transforming conveyor cloud");
	sensor_msgs::PointCloud2 odom_pc2;
  pcl::toROSMsg(*odom_conveyor_cloud, odom_pc2 );
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

	pcl::fromROSMsg(depth_pcl_pc2,*conveyor_cloud_);

	// fit the table plane to get coefficients
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  PointCloudOperations::fitPlanarModel(conveyor_cloud_, inliers, coefficients, pipeline_data->planeMaxIterations, pipeline_data->planeDistanceThreshold);
  logger.logInfo((boost::format("Table inlier count: %s") % inliers->indices.size ()).str());
  logger.logInfo((boost::format("pcl::ModelCoefficients: %s") % coefficients->values.size()).str());
  for (int i = 0; i < coefficients->values.size(); i++)
  {
    logger.logInfo((boost::format("  %s") % coefficients->values[i]).str());
  }
  table_coefficients_ = coefficients;
}

bool 
Task6Segmenter::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
    PipelineData::Ptr &pipeline_data, 
    PipelineObject::VecPtr &pipeline_objects)
{
	if (!transform_success_)
	{
		logger.logError("Initialization failed! call updateConveyorCloud! Can't segment");
		return false;
	}
  boost::posix_time::ptime mps_start = boost::posix_time::microsec_clock::local_time();

	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);

	// debug
  logger.logInfo((boost::format("pcl::ModelCoefficients: %s") % table_coefficients_->values.size()).str());
  for (int i = 0; i < table_coefficients_->values.size(); i++)
  {
    logger.logInfo((boost::format("  %s") % table_coefficients_->values[i]).str());
  }
  
	// self made plane generation
	// TODO: dynamic reconfigure
	double max_plane_dist = 0.002; // 2mm
	double max_object_dist = 0.1; // 10cm
	
	labels->points.resize(cloud->points.size());
	// init height calculation
  float a,b,c,d,e;
  a = table_coefficients_->values[0];
  b = table_coefficients_->values[1];
  c = table_coefficients_->values[2];
  d = table_coefficients_->values[3];
  e = sqrt(a*a + b*b + c*c);
	int table_point_count = 0;
	
	table_pointcloud_ = conveyor_cloud_;
	//table_pointcloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
#pragma omp parallel for
	for (int i = 0; i < cloud->points.size(); i++)
	{
		pcl::PointXYZRGB *p = &cloud->points[i];
		// height calculation
		double tmp = ( a * p->x + b * p->y + c * p->z + d ) / e;
    tmp = tmp < 0 ? -tmp : tmp; // abs
    
		labels->points[i].label = tmp < max_plane_dist || tmp > max_object_dist ? 0 : 1;
		/*
    if (tmp < max_plane_dist || tmp > max_object_dist)
		{
			labels->points[i].label = 0;
			table_pointcloud_->points.push_back(cloud->points[i]);
		}
		else
		{
			labels->points[i].label = 1;
		}
		*/
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
	euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);
	
	pcl::PointCloud<pcl::Label> euclidean_labels;
	std::vector<pcl::PointIndices> euclidean_label_indices;
	pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGB,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
	euclidean_segmentation.setInputCloud (cloud);
	euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
	
	for (size_t i = 0; i < euclidean_label_indices.size (); i++)
	{
		if (euclidean_label_indices[i].indices.size () > 100)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,*cluster);
			clusters.push_back (cluster);
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
  pipeline_data->coefficients_ = table_coefficients_;
	
  boost::posix_time::ptime mps_end = boost::posix_time::microsec_clock::local_time();
	logger.logTime(mps_start, mps_end, "Segmentation took: ");
	
	return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Task6Segmenter::generate_simple_conveyor_cloud()
{
	pcl::PointXYZRGB v_dp;
	v_dp.x = task_.conveyor_belt.drop_center_point.x;
	v_dp.y = task_.conveyor_belt.drop_center_point.y;
	v_dp.z = task_.conveyor_belt.drop_center_point.z;

	pcl::PointXYZRGB v_mdl;
	v_mdl.x = task_.conveyor_belt.move_direction_and_length.x;
	v_mdl.y = task_.conveyor_belt.move_direction_and_length.y;
	v_mdl.z = task_.conveyor_belt.move_direction_and_length.z;

	double w = task_.conveyor_belt.drop_deviation.y * 2;
	
	double clen = sqrt(v_mdl.x * v_mdl.x + v_mdl.y * v_mdl.y);
	double alpha = PI/2 - asin(v_mdl.y / clen);
	double wx = w * cos(alpha);
	double wy = w * sin(alpha);
	
	logger.logInfo((boost::format("conveyor belt description: v_dp = (%s, %s, %s), v_mdl = (%s, %s, %s), w = %s") %
	  task_.conveyor_belt.drop_center_point.x % task_.conveyor_belt.drop_center_point.y % task_.conveyor_belt.drop_center_point.z %
		task_.conveyor_belt.move_direction_and_length.x % task_.conveyor_belt.move_direction_and_length.y %
		task_.conveyor_belt.move_direction_and_length.z % w).str());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr conveyor_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	conveyor_cloud->height = 1;
	conveyor_cloud->width = 4;
	conveyor_cloud->is_dense = false;
	conveyor_cloud->points.resize(4);

	conveyor_cloud->points[0].x = v_dp.x + v_mdl.x + wx;
	conveyor_cloud->points[0].y = v_dp.y + v_mdl.y + wy;
	conveyor_cloud->points[0].z = v_dp.z + v_mdl.z;
	
	conveyor_cloud->points[1].x = v_dp.x + v_mdl.x - wx;
	conveyor_cloud->points[1].y = v_dp.y + v_mdl.y - wy;
	conveyor_cloud->points[1].z = v_dp.z + v_mdl.z;
	
	conveyor_cloud->points[2].x = v_dp.x + wx;
	conveyor_cloud->points[2].y = v_dp.y + wy;
	conveyor_cloud->points[2].z = v_dp.z;
	
	conveyor_cloud->points[3].x = v_dp.x - wx;
	conveyor_cloud->points[3].y = v_dp.y - wy;
	conveyor_cloud->points[3].z = v_dp.z;
	
	return conveyor_cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Task6Segmenter::getTablePointCloud()
{
  return table_pointcloud_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Task6Segmenter::getDownsampledPointCloud()
{
  return downsampled_pointcloud_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Task6Segmenter::getPointsAboveTable()
{
  return points_above_table_;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr Task6Segmenter::getProjectedPoints()
{
  return projected_points_;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Task6Segmenter::getProjectionClusters()
{
  return projection_clusters_;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Task6Segmenter::getProjectionClusterHulls()
{
  return projection_cluster_hulls_;
}
