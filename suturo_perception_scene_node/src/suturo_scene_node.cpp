#include "suturo_scene_node.h"

#include "perception_utils/point_cloud_operations.h"

#include <pcl/filters/passthrough.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <shape_msgs/Plane.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>

using namespace suturo_perception;

SuturoSceneNode::SuturoSceneNode(ros::NodeHandle &n, std::string imageTopic, std::string depthTopic) : 
  nodeHandle_(n), 
  imageTopic_(imageTopic),
  cloudTopic_(depthTopic)
{
	logger = Logger("SuturoPerceptionSceneNode");
  clusterService_ = nodeHandle_.advertiseService("/suturo/GetScene", 
    &SuturoSceneNode::getScene, this);
	idx_ = 0;

  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/suturo/cuboid_markers", 0);
  maxMarkerId_ = 0;

  // Set default parameters
  zAxisFilterMin = 0.0f;
  zAxisFilterMax = 1.5f;
  downsampleLeafSize = 0.01f; // [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow
  planeMaxIterations = 1000;
  planeDistanceThreshold = 0.01f; 
  ecClusterTolerance = 0.02f; // 2cm
  ecMinClusterSize = 1000;
  ecMaxClusterSize = 200000;  
  prismZMin = 0.001f;
  prismZMax = 0.50f; // cutoff 50 cm above plane
  ecObjClusterTolerance = 0.05f; // 3cm
  ecObjMinClusterSize = 10;
  ecObjMaxClusterSize = 25000;
}

void SuturoSceneNode::publish_marker(std::vector<PipelineObject::Ptr> objects)
{
  logger.logInfo("Publishing centroid marker");

  for (int i = 0; i < maxMarkerId_; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/sdepth"; // TODO: more dynamic?
    marker.header.stamp = ros::Time();
    marker.ns = "suturo_perception";
    marker.id = i;
    marker.action = visualization_msgs::Marker::DELETE;
    markerPublisher_.publish(marker);
  }

  for (int i = 0; i < objects.size(); i++)
  {
    PipelineObject::Ptr obj = objects[i];
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/sdepth"; // TODO: more dynamic?
    marker.header.stamp = ros::Time();
    marker.ns = "suturo_perception";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obj->get_c_centroid().x;
    marker.pose.position.y = obj->get_c_centroid().y;
    marker.pose.position.z = obj->get_c_centroid().z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markerPublisher_.publish(marker);
  }

  maxMarkerId_ = objects.size();
}

bool
SuturoSceneNode::getScene(suturo_perception_msgs::GetScene::Request &req, suturo_perception_msgs::GetScene::Response &res)
{
	res.id = idx_;
	idx_++;
	
	ros::Subscriber sub = nodeHandle_.subscribe<sensor_msgs::PointCloud2>(cloudTopic_, 1, boost::bind(&SuturoSceneNode::receive_cloud,this, _1));
	
	logger.logInfo("Waiting for processed cloud");
  ros::Rate r(20); // 20 hz
  // cancel service call, if no cloud is received after 10s
  boost::posix_time::ptime cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(10);
	processing_ = true;
  while(processing_)
  {
    if(boost::posix_time::second_clock::local_time() >= cancelTime)
    {
      processing_ = false;
			logger.logError("No sensor data available. Aborting.");
			return false;
		}
    ros::spinOnce();
    r.sleep();
  }
  logger.logInfo("done with segmentation, starting pipeline");
  
  /*
  if (coefficients_->values.size() != 4)
	{
		logger.logError("coefficients_.size() != 4");
		return false;
	}
	shape_msgs::Plane plane;
	for (int i = 0; i < 4; i++)
	{
		plane.coef[i] = coefficients_->values.at(i);
	}
  moveit_msgs::CollisionObject table_msg;
	table_msg.planes.push_back(plane);
  //res.table = table_msg;
  */

  logger.logInfo("done with perception pipeline, sending result");
  for (int i = 0; i < pipelineObjects_.size(); i++)
  {
    logger.logInfo("sending object");
    if (pipelineObjects_.at(i) == NULL)
    {
      logger.logError("pipeline object is NULL! investigate this!");
      continue;
    }
    res.objects.push_back(pipelineObjects_[i]->toEurocObject());
  }

  logger.logInfo("results sent, publishing markers");
  publish_marker(pipelineObjects_);

  return true;
}

/**
 * Use EuclideanClusterExtraction on object_clusters to identify seperate objects in the given pointcloud.
 * Create a ConvexHull for every object_cluster and extract everything that's above it (in a given range,
 * see SuturoPerception::prismZMax and SuturoPerception::prismZMin.
 * In the future, this method will also extract 2d images from every object cluster.
 */
void SuturoSceneNode::clusterFromProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<pcl::PointIndices::Ptr> &original_indices)
{

  if(object_clusters->points.size() == 0)
  {
    logger.logError("clusterFromProjection: object_clusters is empty. Skipping ...");
    return;
  }

  if(original_cloud->points.size() < 50)
  {
    logger.logError("clusterFromProjection: original_cloud has less than 50 points. Skipping ...");
    return;
  }

  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  // Identify clusters in the input cloud
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (object_clusters);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (ecObjClusterTolerance);
  ec.setMinClusterSize (ecObjMinClusterSize);
  ec.setMaxClusterSize (ecObjMaxClusterSize);
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
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
      cloud_cluster->points.push_back (object_clusters->points[*pit]); //*

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;
    logger.logInfo((boost::format("Cloud Cluster Size is %s") % cloud_cluster->points.size ()).str());
    //std::ostringstream fn;
    //fn << "2dcluster_" << i << ".pcd";
    //if(writer_pcd) writer.write(fn.str(), *cloud_cluster, false);

  
    // Extract every point above the 2d cluster.
    // These points will belong to a single object on the table
    pcl::PointIndices::Ptr object_indices (new pcl::PointIndices); // The extracted indices of a single object above the plane
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_points (new pcl::PointCloud<pcl::PointXYZRGB>());
    PointCloudOperations::extractAllPointsAbovePointCloud(original_cloud, cloud_cluster, object_points, object_indices, 2,
        prismZMin, prismZMax);
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

}


void
SuturoSceneNode::receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
	logger.logInfo("image and cloud incoming...");
  if (!processing_)
  {
    logger.logInfo("processing_ == false, aborting...");
    return;
  }
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*inputCloud,*cloud_in);
  
  

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
  
  logger.logInfo((boost::format("PointCloud after z-filter: %s data points") % cloud_filtered->points.size()).str());

  //voxelizing cloud
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>());
  PointCloudOperations::downsample(cloud_filtered, cloud_downsampled, downsampleLeafSize);
  cloud_filtered = cloud_downsampled; // Use the downsampled cloud now
  
  logger.logInfo((boost::format("PointCloud after downsample: %s data points") % cloud_filtered->points.size()).str());

  // Find the biggest table plane in the scene
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  PointCloudOperations::fitPlanarModel(cloud_filtered, inliers, coefficients, planeMaxIterations, planeDistanceThreshold);
  logger.logInfo((boost::format("Table inlier count: %s") % inliers->indices.size ()).str());
  logger.logInfo((boost::format("pcl::ModelCoefficients: %s") % coefficients->values.size()).str());
  for (int i = 0; i < coefficients->values.size(); i++)
  {
    logger.logInfo((boost::format("  %s") % coefficients->values[i]).str());
  }
  coefficients_ = coefficients;
  
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
  
  // Extract all objects above
  // the table plane
  pcl::PointIndices::Ptr object_indices (new pcl::PointIndices);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters (new pcl::PointCloud<pcl::PointXYZRGB>());
  PointCloudOperations::extractAllPointsAbovePointCloud(cloud_filtered, plane_cluster,
      object_clusters, object_indices, 2, prismZMin, prismZMax);
  //objects_on_plane_cloud_ = object_clusters;

  // Project the pointcloud above the table onto the table to get a 2d representation of the objects
  // This will cause every point of an object to be at the base of the object
  PointCloudOperations::projectToPlaneCoefficients(cloud_filtered, object_indices, coefficients, objects_cloud_projected);
  //if(writer_pcd) writer.write ("objects_cloud_projected.pcd", *objects_cloud_projected, false);

  // Take the projected points, cluster them and extract everything that's above it
  // By doing this, we should get every object on the table and a 2d image of it.
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> extractedObjects;
	std::vector<pcl::PointIndices::Ptr> extractedIndices;
  clusterFromProjection(objects_cloud_projected, cloud_in, &removed_indices_filtered, extractedObjects, extractedIndices);
  logger.logInfo((boost::format(" - extractedObjects Vector size %s") % extractedObjects.size()).str());
  logger.logInfo((boost::format(" - extractedIndices  Vector size %s") % extractedIndices.size()).str());
 
  e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "table from pointcloud");

  for (int ex = 0; ex < extractedObjects.size(); ex++)
  {
  	// write pcd
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << "euroc_cloud_" << idx_ << "_" << ex << ".pcd";
    writer.write(ss.str(), *(extractedObjects[ex]));
    std::cerr << "Saved " << extractedObjects[ex]->points.size () << " data points to " << ss.str().c_str() << std::endl;
  }
  
  pipelineObjects_.clear();
  for (int i = 0; i < extractedObjects.size(); i++)
  {
    PipelineObject::Ptr pipelineObject(new PipelineObject);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr it = extractedObjects.at(i);
    logger.logInfo((boost::format("Transform cluster %s into a message. \
                    Cluster has %s points") % i % it->points.size()).str());

    if(it->points.size()==0)
    {
      logger.logError("Cluster cloud is empty. Skipping ...");
      i++;
      continue;
    }

    if(it->points.size()<50)
    {
      logger.logError("Cluster cloud has less than 50 points. Skipping ...");
      i++;
      continue;
    }

    pipelineObject->set_pointCloud(it);
    
    // Calculate the volume of each cluster
    // Create a convex hull around the cluster and calculate the total volume
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_points_from_hull (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(it);
    hull.setDimension(3);
    hull.setComputeAreaVolume(true); // This creates alot of output, but it's necessary for getTotalVolume() ....
    hull.reconstruct (*hull_points);

    // Centroid calulcation
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*hull_points, centroid);  

    logger.logInfo((boost::format("Centroid: %s, %s, %s") % centroid[0] % centroid[1] % centroid[2]).str());

		Point centroid_point;
    centroid_point.x = centroid[0];
    centroid_point.y = centroid[1];
    centroid_point.z = centroid[2];
    pipelineObject->set_c_centroid(centroid_point);
		
    pipelineObjects_.push_back(pipelineObject);
	}


  idx_++;
	processing_ = false;
}
