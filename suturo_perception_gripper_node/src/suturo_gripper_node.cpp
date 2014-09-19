#include "suturo_gripper_node.h"

#include <perception_utils/point_cloud_operations.h>
#include <perception_utils/publisher_helper.h>
#include <perception_utils/get_euroc_task_description.h>
#include <suturo_perception_segmentation/projection_segmenter.h>
#include <suturo_perception_pipeline/pipeline.h>
#include <suturo_perception_msgs/EurocObject.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace suturo_perception;

const std::string SuturoGripperNode::OBJECT_CLOUD_PREFIX_TOPIC= "/suturo/object_cluster_cloud/";
const std::string SuturoGripperNode::TABLE_TOPIC= "/suturo/tcp_table/";
const std::string SuturoGripperNode::DOWNSAMPLED_CLOUD= "/suturo/tcp_downsampled_cloud/";
const std::string SuturoGripperNode::POINTS_ABOVE_TABLE_CLOUD= "/suturo/tcp/points_above_table/";
const std::string SuturoGripperNode::PROJECTED_POINTS_TOPIC= "/suturo/tcp/projected_points/";
const std::string SuturoGripperNode::PROJECTED_CLUSTERS_PREFIX_TOPIC= "/suturo/tcp/projected_point_clusters/";
const std::string SuturoGripperNode::PROJECTED_CLUSTER_HULLS_PREFIX_TOPIC= "/suturo/tcp/projected_point_hulls/";

SuturoGripperNode::SuturoGripperNode(ros::NodeHandle &n, std::string imageTopic, std::string depthTopic) : 
  nodeHandle_(n), 
  imageTopic_(imageTopic),
  cloudTopic_(depthTopic),
  ph_(n)
{
	logger = Logger("SuturoPerceptionGripperNode");
  clusterService_ = nodeHandle_.advertiseService("/suturo/GetGripper", 
    &SuturoGripperNode::getGripper, this);
	idx_ = 0;
  objidx_ = 0;

  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/suturo/cuboid_markers_gripper", 0);
  maxMarkerId_ = 0;

  // Add additional topics for debugging purposes
  // init 7 topics for the pointclouds of every object cluster and projection cluster
  for(int i = 0; i <= 6; ++i)
  {
    std::stringstream ss;
    ss << i;
    ph_.advertise<sensor_msgs::PointCloud2>(OBJECT_CLOUD_PREFIX_TOPIC + ss.str());
    ph_.advertise<sensor_msgs::PointCloud2>(PROJECTED_CLUSTERS_PREFIX_TOPIC + ss.str());
    ph_.advertise<sensor_msgs::PointCloud2>(PROJECTED_CLUSTER_HULLS_PREFIX_TOPIC + ss.str());
  }
  ph_.advertise<sensor_msgs::PointCloud2>(TABLE_TOPIC);
  ph_.advertise<sensor_msgs::PointCloud2>(DOWNSAMPLED_CLOUD);
  ph_.advertise<sensor_msgs::PointCloud2>(POINTS_ABOVE_TABLE_CLOUD);
  ph_.advertise<sensor_msgs::PointCloud2>(PROJECTED_POINTS_TOPIC);
  
  // Initialize pipeline configuration
  pipelineData_ = PipelineData::Ptr(new PipelineData());
  
  // Initialize dynamic reconfigure
  reconfCb = boost::bind(&SuturoGripperNode::reconfigureCallback, this, _1, _2);
  reconfSrv.setCallback(reconfCb);

}

/*
 * Callback for the dynamic reconfigure service
 */
void SuturoGripperNode::reconfigureCallback(suturo_perception_gripper_node::SuturoPerceptionConfig &config, uint32_t level)
{
  pipelineData_->zAxisFilterMin = config.zAxisFilterMin;
  pipelineData_->zAxisFilterMax = config.zAxisFilterMax;
  pipelineData_->downsampleLeafSize = config.downsampleLeafSize;
  pipelineData_->planeMaxIterations = config.planeMaxIterations;
  pipelineData_->planeDistanceThreshold = config.planeDistanceThreshold;
  pipelineData_->ecClusterTolerance = config.ecClusterTolerance;
  pipelineData_->ecMinClusterSize = config.ecMinClusterSize;
  pipelineData_->ecMaxClusterSize = config.ecMaxClusterSize;
  pipelineData_->prismZMin = config.prismZMin;
  pipelineData_->prismZMax = config.prismZMax;
  pipelineData_->ecObjClusterTolerance = config.ecObjClusterTolerance;
  pipelineData_->ecObjMinClusterSize = config.ecObjMinClusterSize;
  pipelineData_->ecObjMaxClusterSize = config.ecObjMaxClusterSize;
  
  pipelineData_->printConfig();
}

bool
SuturoGripperNode::getGripper(suturo_perception_msgs::GetGripper::Request &req, suturo_perception_msgs::GetGripper::Response &res)
{
	res.id = idx_;
	idx_++;

	pipelineData_->resetData();
  pipelineData_->request_parameters_ = req.s;
  
  // get task description
  EurocTaskClient task_client(nodeHandle_);
  logger.logInfo("Requesting task description");
  if (!task_client.requestTaskDescription())
  {
    logger.logError("Requesting task description failed. Aborting.");
    return false;
  }
  pipelineData_->task_ = task_client.getTaskDescription();
  
	ros::Subscriber sub = nodeHandle_.subscribe<sensor_msgs::PointCloud2>(cloudTopic_, 1, boost::bind(&SuturoGripperNode::receive_cloud,this, _1));
	
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
    
  /****************************************************************************/
  Pipeline::execute(pipelineData_, pipelineObjects_);
  /****************************************************************************/

  logger.logInfo("done with perception pipeline, sending result");
  for (int i = 0; i < pipelineObjects_.size(); i++)
  {
    logger.logInfo("sending object");
    if (pipelineObjects_.at(i) == NULL)
    {
      logger.logError("pipeline object is NULL! investigate this!");
      continue;
    }
    suturo_perception_msgs::EurocObject euObj = pipelineObjects_[i]->toEurocObject();
    euObj.frame_id = "/tdepth_pcl";
    euObj.c_id = objidx_;
    objidx_++;
    euObj.object.header.frame_id="/tdepth_pcl";
    res.objects.push_back(euObj);
  }

  logger.logInfo("results sent, publishing markers");
  PublisherHelper::publish_marker(pipelineObjects_, "/tdepth_pcl", markerPublisher_, &maxMarkerId_);

  logger.logInfo("Publishing data on debugging topics");
  for (int i = 0; i < pipelineObjects_.size(); i++)
  {
    std::stringstream ss;
    ss << i;
    ph_.publish_pointcloud(OBJECT_CLOUD_PREFIX_TOPIC + ss.str(), 
        pipelineObjects_[i]->get_pointCloud(), "/tdepth_pcl");
  }

  return true;
}


void
SuturoGripperNode::receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
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
  
  
  ProjectionSegmenter projection_segmenter;
  if (!projection_segmenter.segment(cloud_in, pipelineData_, pipelineObjects_))
  {
    logger.logInfo("segmentation failed");
    return;
  }
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projected_points_clusters =
    projection_segmenter.getProjectionClusters();
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projected_point_hulls =
    projection_segmenter.getProjectionClusterHulls();
  for (int i = 0; i < projected_points_clusters.size(); i++)
  {
    std::stringstream ss;
    ss << i;
    ph_.publish_pointcloud(PROJECTED_CLUSTERS_PREFIX_TOPIC + ss.str(), 
        projected_points_clusters[i], "/tdepth_pcl");
    ph_.publish_pointcloud(PROJECTED_CLUSTER_HULLS_PREFIX_TOPIC + ss.str(), 
        projected_point_hulls[i], "/tdepth_pcl");
  }
  // Publish the segmentation debug topics
  ph_.publish_pointcloud(TABLE_TOPIC, projection_segmenter.getTablePointCloud()
        , "/tdepth_pcl");

  ph_.publish_pointcloud(DOWNSAMPLED_CLOUD, projection_segmenter.getDownsampledPointCloud()
        , "/tdepth_pcl");
  ph_.publish_pointcloud(POINTS_ABOVE_TABLE_CLOUD, projection_segmenter.getPointsAboveTable()
        , "/tdepth_pcl");

  ph_.publish_pointcloud(PROJECTED_POINTS_TOPIC, projection_segmenter.getProjectedPoints()
        , "/tdepth_pcl");

	processing_ = false;
}
