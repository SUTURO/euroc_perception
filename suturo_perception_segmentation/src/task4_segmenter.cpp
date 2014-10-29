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
	}
	else
	{
		pcl::fromROSMsg(depth_pcl_pc2,*segmentation_cloud_);
	}
	
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

void
Task4Segmenter::cloud_cb (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clusters, PipelineData::Ptr &pipeline_data)
{
}

bool 
Task4Segmenter::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
    PipelineData::Ptr &pipeline_data, 
    PipelineObject::VecPtr &pipeline_objects)
{
	/*
	pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> mps;
	
	// Set up Normal Estimation
	//ne.setNormalEstimationMethod (ne.SIMPLE_3D_GRADIENT);
	ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
	ne.setMaxDepthChangeFactor (0.005f);
	ne.setNormalSmoothingSize (20.0f);
	//
	pcl::PlaneCoefficientComparator<pcl::PointXYZRGB, pcl::Normal>::Ptr plane_comparator_(new pcl::PlaneCoefficientComparator<pcl::PointXYZRGB, pcl::Normal> ());
	pcl::EuclideanPlaneCoefficientComparator<pcl::PointXYZRGB, pcl::Normal>::Ptr euclidean_comparator_(new pcl::EuclideanPlaneCoefficientComparator<pcl::PointXYZRGB, pcl::Normal> ());
	pcl::RGBPlaneCoefficientComparator<pcl::PointXYZRGB, pcl::Normal>::Ptr rgb_comparator_(new pcl::RGBPlaneCoefficientComparator<pcl::PointXYZRGB, pcl::Normal> ());
	pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB, pcl::Normal>::Ptr edge_aware_comparator_(new pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB, pcl::Normal> ());
	
	// Set up Organized Multi Plane Segmentation
	mps.setMinInliers (10000);
	mps.setAngularThreshold (pcl::deg2rad (1.0)); //3 degrees
	mps.setDistanceThreshold (0.005); //5mm

	//QMutexLocker locker (&mtx_);
	//FPS_CALC ("computation");
	// Estimate Normals
	pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
	ne.setInputCloud (cloud);
	ne.compute (*normal_cloud);
	float* distance_map = ne.getDistanceMap ();
	boost::shared_ptr<pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB,pcl::Normal> > eapc = boost::dynamic_pointer_cast<pcl::EdgeAwarePlaneComparator<pcl::PointXYZRGB,pcl::Normal> >(edge_aware_comparator_);
	eapc->setDistanceMap (distance_map);
	eapc->setDistanceThreshold (0.01f, false);
	
	// Segment Planes
	//double mps_start = pcl::getTime ();
  boost::posix_time::ptime mps_start = boost::posix_time::microsec_clock::local_time();
	std::vector<pcl::PlanarRegion<pcl::PointXYZRGB>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZRGB> > > regions;
	std::vector<pcl::ModelCoefficients> model_coefficients;
	std::vector<pcl::PointIndices> inlier_indices;
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	std::vector<pcl::PointIndices> label_indices;
	std::vector<pcl::PointIndices> boundary_indices;
	mps.setInputNormals (normal_cloud);
	mps.setInputCloud (cloud);
	bool use_planar_refinement_ = true;
	if (use_planar_refinement_)
	{
		mps.segmentAndRefine (regions, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
		for (int i = 0; i < model_coefficients.size(); i++)
		{
			logger.logInfo((boost::format("%s: pcl::ModelCoefficients: %s") % i % coefficients->values.size()).str());
			for (int j = 0; j < model_coefficients.at(i).values.size(); j++)
			{
				logger.logInfo((boost::format("  %s") % model_coefficients.at(i).values[j]).str());
			}
		}
	}
	else
	{
		mps.segment (regions);//, model_coefficients, inlier_indices, labels, label_indices, boundary_indices);
	}
	//double mps_end = pcl::getTime ();
  boost::posix_time::ptime mps_end = boost::posix_time::microsec_clock::local_time();
	logger.logTime(mps_start, mps_end, "MPS+Refine took: ");
	*/
	
	pcl::PointCloud<pcl::Label>::Ptr labels (new pcl::PointCloud<pcl::Label>);
	//std::vector<pcl::PointIndices> label_indices;
	
	// self made plane generation
	double max_plane_dist = 0.002; // 2mm
	
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
	//bool use_clustering_ = true;
	//if (use_clustering_ && regions.size () > 0)
	//{
	std::vector<bool> plane_labels;
	//plane_labels.resize (label_indices.size (), false);
	plane_labels.push_back(true);
	plane_labels.push_back(false);
	//for (size_t i = 0; i < label_indices.size (); i++)
	//{
	//	if (label_indices[i].indices.size () > 10000)
	//	{
	//		plane_labels[i] = true;
	//	}
	//}
	
	pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr  euclidean_cluster_comparator_ = pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label>::Ptr (new pcl::EuclideanClusterComparator<pcl::PointXYZRGB, pcl::Normal, pcl::Label> ());
	euclidean_cluster_comparator_->setInputCloud (cloud);
	euclidean_cluster_comparator_->setLabels (labels);
	euclidean_cluster_comparator_->setExcludeLabels (plane_labels);
	//euclidean_cluster_comparator_->setExcludeLabels (empty_plane_labels);
	euclidean_cluster_comparator_->setDistanceThreshold (0.01f, false);
	pcl::PointCloud<pcl::Label> euclidean_labels;
	std::vector<pcl::PointIndices> euclidean_label_indices;
	pcl::OrganizedConnectedComponentSegmentation<pcl::PointXYZRGB,pcl::Label> euclidean_segmentation (euclidean_cluster_comparator_);
	euclidean_segmentation.setInputCloud (cloud);
	euclidean_segmentation.segment (euclidean_labels, euclidean_label_indices);
	for (size_t i = 0; i < euclidean_label_indices.size (); i++)
	{
		if (euclidean_label_indices[i].indices.size () > 1000)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::copyPointCloud (*cloud,euclidean_label_indices[i].indices,*cluster);
			clusters.push_back (cluster);
			logger.logInfo((boost::format("euclidean cluster %s has %s points!") % i % cluster->points.size()).str());
		}
	}
	logger.logInfo((boost::format("Got %s euclidean clusters!") % clusters.size() ).str());
	projection_clusters_ = clusters;
	//}

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
Task4Segmenter::segment_old(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
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
