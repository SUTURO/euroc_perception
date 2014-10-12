#include "suturo_perception_segmentation/projection_segmenter.h"

#include <perception_utils/point_cloud_operations.h>
#include <pcl/point_types.h>

using namespace suturo_perception;

bool ProjectionSegmenter::clusterPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<pcl::PointIndices::Ptr> &original_indices, PipelineData::Ptr &pipeline_data)
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
ProjectionSegmenter::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
    PipelineData::Ptr &pipeline_data, 
    PipelineObject::VecPtr &pipeline_objects)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>), 
                                      cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      objects_cloud_projected (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB>),
                                      cloud_without_plane (new pcl::PointCloud<pcl::PointXYZRGB>);

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

  // Find the biggest table plane in the scene
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  PointCloudOperations::fitPlanarModel(cloud_filtered, inliers, coefficients, pipeline_data->planeMaxIterations, pipeline_data->planeDistanceThreshold);
  logger.logInfo((boost::format("Table inlier count: %s") % inliers->indices.size ()).str());
  logger.logInfo((boost::format("pcl::ModelCoefficients: %s") % coefficients->values.size()).str());
  for (int i = 0; i < coefficients->values.size(); i++)
  {
    logger.logInfo((boost::format("  %s") % coefficients->values[i]).str());
  }
  pipeline_data->coefficients_ = coefficients;
	logger.logWarn((boost::format("task_type = %d") % pipeline_data->task_.task_type).str());
	logger.logWarn((boost::format("task_name = %s") % pipeline_data->task_.task_name).str());
  if (pipeline_data->task_.task_type == 5)
	{
		logger.logWarn("Doing special stuff for task 6...");
		PointCloudOperations::extractInliersFromPointCloud(cloud_filtered, inliers, cloud_without_plane, true);
		// Find the second biggest table plane in the scene
		//pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		//pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
		PointCloudOperations::fitPlanarModel(cloud_without_plane, inliers, coefficients, pipeline_data->planeMaxIterations, pipeline_data->planeDistanceThreshold);
		logger.logInfo((boost::format("Second Table inlier count: %s") % inliers->indices.size ()).str());
		logger.logInfo((boost::format("pcl::ModelCoefficients of second plane: %s") % coefficients->values.size()).str());
		for (int i = 0; i < coefficients->values.size(); i++)
		{
			logger.logInfo((boost::format("  %s") % coefficients->values[i]).str());
		}
		pipeline_data->coefficients_ = coefficients;
	}

  // Extract the plane as a PointCloud from the calculated inliers
  PointCloudOperations::extractInliersFromPointCloud(cloud_filtered, inliers, cloud_plane, false);

  // Take the biggest cluster in the extracted plane. This will be
  // most likely our desired table pointcloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointIndices::Ptr new_inliers (new pcl::PointIndices);
  PointCloudOperations::extractBiggestCluster(cloud_plane, plane_cluster, inliers, new_inliers,
    pipeline_data->ecObjClusterTolerance, pipeline_data->ecMinClusterSize, pipeline_data->ecMaxClusterSize);
  // Save the plane cluster for debugging purposes in a member variable
  table_pointcloud_ = plane_cluster;

  // NOTE: We need to transform the inliers from table_cluster_indices to inliers
  inliers = new_inliers;
  
  if(inliers->indices.size () == 0)
  {
    logger.logError("Second Table Inlier Set is empty. Exiting....");
    return false;
  }
  
  // Extract all objects above
  // the table plane
  pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projected_clusters;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projected_cluster_hulls;

  // Remove all NaNs from the PointCloud. Otherwise, we can't use Euclidean Clustering (KDTree)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr nanles_cloud (new pcl::PointCloud<pcl::PointXYZRGB>()); // NEW
  PointCloudOperations::removeNans(cloud_in, nanles_cloud); // NEW

  // PointCloudOperations::extractAllPointsAbovePointCloud(cloud_filtered, plane_cluster, // OLD
  PointCloudOperations::extractAllPointsAbovePointCloud(nanles_cloud, plane_cluster, // New
      object_clusters, object_indices, 2, pipeline_data->prismZMin, pipeline_data->prismZMax);
  logger.logInfo((boost::format("After extractAllPointsAbovePointCloud: %s indices and %s object_cluster pts") % object_indices->indices.size() % object_clusters->points.size() ).str() );
  //objects_on_plane_cloud_ = object_clusters;
  points_above_table_ = object_clusters;

  // Project the pointcloud above the table onto the table to get a 2d representation of the objects
  // This will cause every point of an object to be at the base of the object
  // PointCloudOperations::projectToPlaneCoefficients(cloud_filtered, object_indices, coefficients, objects_cloud_projected); // OLD - Not necessary for "workaround" segmentation
  projected_points_ = objects_cloud_projected;

  // Take the projected points, cluster them and extract everything that's above it
  // By doing this, we should get every object on the table and a 2d image of it.
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects;
	std::vector<pcl::PointIndices::Ptr> extractedIndices;
  // clusterFromProjection(objects_cloud_projected, cloud_in, &removed_indices_filtered, extractedObjects, extractedIndices, projected_clusters, projected_cluster_hulls, pipeline_data); // OLD. Extract PointClouds by using the 2d Projection clusters.

  clusterPointcloud(object_clusters, extractedObjects, extractedIndices, pipeline_data); // NEW - Just cluster everything above the table - This is unfortunately slower then the projection method ...
  logger.logInfo((boost::format(" - extractedObjects Vector size %s") % extractedObjects.size()).str());
  logger.logInfo((boost::format(" - extractedIndices  Vector size %s") % extractedIndices.size()).str());
  projection_clusters_ = projected_clusters;
  projection_cluster_hulls_ = projected_cluster_hulls;
 
  e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "table from pointcloud");

  /*
  for (int ex = 0; ex < extractedObjects.size(); ex++)
  {
  	// write pcd
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << "euroc_cloud_" << idx_ << "_" << ex << ".pcd";
    writer.write(ss.str(), *(extractedObjects[ex]));
    std::cerr << "Saved " << extractedObjects[ex]->points.size () << " data points to " << ss.str().c_str() << std::endl;
  }
  */


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

  return true;
}


/**
 * Use EuclideanClusterExtraction on object_clusters to identify seperate objects in the given pointcloud.
 * Create a ConvexHull for every object_cluster and extract everything that's above it (in a given range,
 * see SuturoPerception::prismZMax and SuturoPerception::prismZMin.
 * In the future, this method will also extract 2d images from every object cluster.
 */
bool ProjectionSegmenter::clusterFromProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<pcl::PointIndices::Ptr> &original_indices, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clustered_projections, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clustered_hulls, PipelineData::Ptr &pipeline_data)
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProjectionSegmenter::getTablePointCloud()
{
  return table_pointcloud_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProjectionSegmenter::getDownsampledPointCloud()
{
  return downsampled_pointcloud_;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProjectionSegmenter::getPointsAboveTable()
{
  return points_above_table_;
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ProjectionSegmenter::getProjectedPoints()
{
  return projected_points_;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ProjectionSegmenter::getProjectionClusters()
{
  return projection_clusters_;
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> ProjectionSegmenter::getProjectionClusterHulls()
{
  return projection_cluster_hulls_;
}
