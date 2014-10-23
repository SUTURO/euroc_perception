#include "suturo_perception_segmentation/task6_segmenter.h"

#include <perception_utils/point_cloud_operations.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/impl/transforms.hpp>
#include <cmath>

#define PI 3.14159265

using namespace suturo_perception;


Task6Segmenter::Task6Segmenter(ros::NodeHandle &node, bool isTcp, suturo_msgs::Task task) 
: Segmenter(), nodeHandle_(node), isTcp_(isTcp), task_(task)
{
	logger = Logger("task6_segmenter");
	
	updateConveyorCloud();
}

void Task6Segmenter::updateConveyorCloud()
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
	}
	else
	{
		pcl::fromROSMsg(depth_pcl_pc2,*conveyor_cloud_);
	}
}

bool Task6Segmenter::clusterPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<pcl::PointIndices::Ptr> &original_indices, PipelineData::Ptr &pipeline_data)
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
Task6Segmenter::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
    PipelineData::Ptr &pipeline_data, 
    PipelineObject::VecPtr &pipeline_objects)
{
	if (!transform_success_)
	{
		logger.logError("transform of conveyor cloud failed, segmentation won't work. Please restart the node");
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
  
  logger.logInfo((boost::format("conveyor_cloud_ has %s points") % conveyor_cloud_->points.size()).str());

  // Find the biggest table plane in the scene
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  PointCloudOperations::fitPlanarModel(conveyor_cloud_, inliers, coefficients, pipeline_data->planeMaxIterations, pipeline_data->planeDistanceThreshold);
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

	PointCloudOperations::extractAllPointsAbovePointCloud(nanles_cloud, conveyor_cloud_, // New
			object_clusters, object_indices, 2, pipeline_data->prismZMin, pipeline_data->prismZMax);
	logger.logInfo((boost::format("After extractAllPointsAbovePointCloud: %s indices and %s object_cluster pts") % object_indices->indices.size() % object_clusters->points.size() ).str() );
	points_above_table_ = object_clusters;
	
	e = boost::posix_time::microsec_clock::local_time();
	logger.logTime(s, e, "extractAllPointsAbovePointCloud");

	// Take the projected points, cluster them and extract everything that's above it
	// By doing this, we should get every object on the table and a 2d image of it.
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects;
	std::vector<pcl::PointIndices::Ptr> extractedIndices;
	clusterPointcloud(object_clusters, extractedObjects, extractedIndices, pipeline_data); // NEW - Just cluster everything above the table - This is unfortunately slower then the projection method ...
	logger.logInfo((boost::format(" - extractedObjects Vector size %s") % extractedObjects.size()).str());
	logger.logInfo((boost::format(" - extractedIndices  Vector size %s") % extractedIndices.size()).str());

	e = boost::posix_time::microsec_clock::local_time();
	logger.logTime(s, e, "table from pointcloud");

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

	table_pointcloud_ = conveyor_cloud_;
	
	e = boost::posix_time::microsec_clock::local_time();
	logger.logTime(s, e, "full segmentation");

  return true;
}


/**
 * Use EuclideanClusterExtraction on object_clusters to identify seperate objects in the given pointcloud.
 * Create a ConvexHull for every object_cluster and extract everything that's above it (in a given range,
 * see SuturoPerception::prismZMax and SuturoPerception::prismZMin.
 * In the future, this method will also extract 2d images from every object cluster.
 */
bool Task6Segmenter::clusterFromProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<pcl::PointIndices::Ptr> &original_indices, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clustered_projections, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clustered_hulls, PipelineData::Ptr &pipeline_data)
{

  if(original_cloud->points.size() < 50)
  {
    logger.logError("clusterFromProjection: original_cloud has less than 50 points. Skipping ...");
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
    logger.logInfo((boost::format("Cloud Cluster Size is %s") % cloud_cluster->points.size ()).str());

    clustered_projections.push_back(cloud_cluster);
    //std::ostringstream fn;
    //fn << "2dcluster_" << i << ".pcd";
    //if(writer_pcd) writer.write(fn.str(), *cloud_cluster, false);

  
    // Extract every point above the 2d cluster.
    // These points will belong to a single object on the table
    pcl::PointIndices::Ptr object_indices (new pcl::PointIndices); // The extracted indices of a single object above the plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points (new pcl::PointCloud<pcl::PointXYZRGB>());
    PointCloudOperations::extractAllPointsAbovePointCloud(original_cloud, cloud_cluster, object_points,
        object_indices, cloud_hull, 2,
        pipeline_data->prismZMin, pipeline_data->prismZMax);

    clustered_hulls.push_back(cloud_hull);
    extracted_objects.push_back(object_points);
		original_indices.push_back(object_indices);

    // logger.logError("After extract");

    boost::posix_time::ptime e1 = boost::posix_time::microsec_clock::local_time();
    logger.logTime(s1, e1, "Extracted Object Points");

    //std::ostringstream cl_file;
    //cl_file << "2d_Z_cluster_" << i << ".pcd";
    //if(writer_pcd) writer.write(cl_file.str(), *object_points, false);
    // cout << "2d_Z_cluster_" << i << " has " << object_points->size() << " points" << endl;

    boost::posix_time::ptime s2 = boost::posix_time::microsec_clock::local_time();

    i++;
  }

  //if(writer_pcd) writer.write ("cluster_from_projection_clusters.pcd", *object_clusters, false);

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
