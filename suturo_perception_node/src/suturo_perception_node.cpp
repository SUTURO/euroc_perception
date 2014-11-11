#include "suturo_perception_node.h"

#include <perception_utils/point_cloud_operations.h>
#include <perception_utils/publisher_helper.h>
#include <suturo_perception_pipeline/pipeline.h>
#include <suturo_perception_msgs/EurocObject.h>
#include <suturo_msgs/Task.h>
#include <suturo_perception_classification/task6_classification.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl/io/pcd_io.h>

using namespace suturo_perception;
SuturoPerceptionNode::SuturoPerceptionNode(ros::NodeHandle &n, std::string imageTopic, std::string depthTopic, NodeType nodeType) : 
  nodeHandle_(n), 
  imageTopic_(imageTopic),
  cloudTopic_(depthTopic),
  nodeType_(nodeType),
  ph_(n)
{
	unsigned char status_node_type;
  switch (nodeType_) 
  {
    case GRIPPER:
      OBJECT_CLOUD_PREFIX_TOPIC= "/suturo/tcp/object_cluster_cloud/";
      TABLE_TOPIC= "/suturo/tcp/table/";
      DOWNSAMPLED_CLOUD= "/suturo/tcp/downsampled_cloud/";
      POINTS_ABOVE_TABLE_CLOUD= "/suturo/tcp/points_above_table/";
      PROJECTED_POINTS_TOPIC= "/suturo/tcp/projected_points/";
      PROJECTED_CLUSTERS_PREFIX_TOPIC= "/suturo/tcp/projected_point_clusters/";
      PROJECTED_CLUSTER_HULLS_PREFIX_TOPIC= "/suturo/tcp/projected_point_hulls/";
      LOGGER_NAME = "SuturoPerceptionGripperNode";
      SERVICE_NAME = "/suturo/GetGripper";
      MARKER_TOPIC = "/suturo/tcp/cuboid_markers_gripper";
      DEPTH_FRAME = "/tdepth_pcl";
			status_node_type = suturo_perception_msgs::PerceptionNodeStatus::NODE_GRIPPER;
    break;
    case SCENE:
      OBJECT_CLOUD_PREFIX_TOPIC= "/suturo/scene/object_cluster_cloud/";
      TABLE_TOPIC= "/suturo/scene/table/";
      DOWNSAMPLED_CLOUD= "/suturo/scene/downsampled_cloud/";
      POINTS_ABOVE_TABLE_CLOUD= "/suturo/scene/points_above_table/";
      PROJECTED_POINTS_TOPIC= "/suturo/scene/projected_points/";
      PROJECTED_CLUSTERS_PREFIX_TOPIC= "/suturo/scene/projected_point_clusters/";
      PROJECTED_CLUSTER_HULLS_PREFIX_TOPIC= "/suturo/scene/projected_point_hulls/";
      LOGGER_NAME = "SuturoPerceptionSceneNode";
      SERVICE_NAME = "/suturo/GetScene";
      MARKER_TOPIC = "/suturo/scene/cuboid_markers_gripper";
      DEPTH_FRAME = "/sdepth_pcl";
			status_node_type = suturo_perception_msgs::PerceptionNodeStatus::NODE_SCENE;
    break;
  }


	logger = Logger(LOGGER_NAME);
  clusterService_ = nodeHandle_.advertiseService(SERVICE_NAME,
    &SuturoPerceptionNode::getGripper, this);
	idx_ = 0;
  objidx_ = 0;

  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>(MARKER_TOPIC, 0);
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
  reconfCb = boost::bind(&SuturoPerceptionNode::reconfigureCallback, this, _1, _2);
  reconfSrv.setCallback(reconfCb);
	
  // get task description
  task_client_ = new EurocTaskClient(nodeHandle_);
  logger.logInfo("Requesting task description");
  if (!task_client_->requestTaskDescription())
  {
    logger.logError("Requesting task description failed. Aborting.");
    //return false;
  }
  
  if (task_client_->getTaskDescription().task_type == suturo_msgs::Task::TASK_6)
	{
		task6_segmenter_ = new Task6Segmenter(nodeHandle_, nodeType_==GRIPPER, task_client_->getTaskDescription());
	}
	else
	{
		task4_segmenter_ = new Task4Segmenter(nodeHandle_, nodeType_==GRIPPER, task_client_->getTaskDescription());
	}
	
	node_status = boost::shared_ptr<NodeStatus> (new NodeStatus(nodeHandle_));
	node_status->nodeStarted(status_node_type);
}

/*
 * Callback for the dynamic reconfigure service
 */
void SuturoPerceptionNode::reconfigureCallback(suturo_perception_node::SuturoPerceptionConfig &config, uint32_t level)
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
  pipelineData_->mpeMaxICPIterations = config.mpeMaxICPIterations;
  pipelineData_->mpeSuccessThreshold = config.mpeSuccessThreshold;
  pipelineData_->mpeVoxelSize = config.mpeVoxelSize;
  pipelineData_->mpeDumpICPFitterPointClouds = config.mpeDumpICPFitterPointClouds;
  pipelineData_->segMaxObjectDistanz = config.segMaxObjectDistanz;

  pipelineData_->printConfig();
}

bool
SuturoPerceptionNode::getGripper(suturo_perception_msgs::GetGripper::Request &req, suturo_perception_msgs::GetGripper::Response &res)
{
	int timeout = 10;
	
	res.id = idx_;
	idx_++;

	pipelineData_->resetData();
  pipelineData_->request_parameters_ = req.s;
  
  pipelineData_->task_ = task_client_->getTaskDescription();
	
	if (task_client_->getTaskDescription().task_type == suturo_msgs::Task::TASK_6)
	{
		if (req.s.find("firstConveyorCall")!=std::string::npos)
		{
			task6_segmenter_->updateConveyorCloud(pipelineData_);
		}
	}
	else
	{
		task4_segmenter_->updateSegmentationCloud(pipelineData_);
	}
  
	ros::Subscriber sub = nodeHandle_.subscribe<sensor_msgs::PointCloud2>(cloudTopic_, 1, boost::bind(&SuturoPerceptionNode::receive_cloud,this, _1));

	logger.logInfo((boost::format("Waiting for processed cloud for %s seconds") % timeout).str());
  ros::Rate r(20); // 20 hz
  // cancel service call, if no cloud is received after 20s
  boost::posix_time::ptime cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(timeout);
	processing_ = true;
  while(processing_)
  {
    if(boost::posix_time::second_clock::local_time() >= cancelTime)
    {
      processing_ = false;
			logger.logError("Timeout reached... No sensor data available. Aborting.");
			return false;
		}
    ros::spinOnce();
    r.sleep();
  }
  
	if (task_client_->getTaskDescription().task_type == suturo_msgs::Task::TASK_4 && 
			nodeType_ == GRIPPER)
	{
		logger.logInfo("Segmenting without timeout (task4 gripper)");
		segment(cloud_in_);
	}

  logger.logInfo("done with segmentation, starting pipeline");

  /****************************************************************************/
  Pipeline::execute(pipelineData_, pipelineObjects_);
  /****************************************************************************/

  logger.logInfo("done with perception pipeline, sending result");
  for (int i = 0; i < pipelineObjects_.size(); i++)
  {
    if (pipelineObjects_.at(i) == NULL)
    {
      logger.logError("pipeline object is NULL! investigate this!");
      continue;
    }
    suturo_perception_msgs::EurocObject euObj = pipelineObjects_[i]->toEurocObject();
		if (pipelineData_->task_.task_type == suturo_msgs::Task::TASK_6)
		{
			logger.logInfo("Filtering the objects with Task6Classification");
			if (!Task6Classification::validObject(pipelineObjects_[i], pipelineData_->task_))
			{
				logger.logInfo("skipping pipeline object! object won't be published!");
				continue;
			}
		}
    std::stringstream ss;
    ss << "sending object ";
    ss << pipelineObjects_.at(i)->get_c_id();
    ss << " (mpe_success = ";
    ss << pipelineObjects_.at(i)->get_c_mpe_success();
    ss << ") END";
    logger.logInfo(ss.str());
		
    euObj.frame_id = DEPTH_FRAME;
    euObj.c_id = objidx_;
    objidx_++;
    euObj.object.header.frame_id = DEPTH_FRAME;
  
    std::stringstream ss2;
    ss2 << "Corresponding EuRoCObject ";
    ss2 << euObj.c_id;
    ss2 << " (mpe_success = ";
    if(euObj.mpe_success)
    {
      ss2 << "true";
    }
    else
    {
      ss2 << "false";
    }
    ss2 << ") END";
    logger.logInfo(ss2.str());
    res.objects.push_back(euObj);
  }

  logger.logInfo("results sent, publishing markers");
  PublisherHelper::publish_marker(pipelineObjects_, DEPTH_FRAME, markerPublisher_, &maxMarkerId_);

  logger.logInfo("Publishing data on debugging topics");
  for (int i = 0; i < pipelineObjects_.size(); i++)
  {
    std::stringstream ss;
    ss << i;
    ph_.publish_pointcloud(OBJECT_CLOUD_PREFIX_TOPIC + ss.str(), 
        pipelineObjects_[i]->get_pointCloud(), DEPTH_FRAME);
  }
  
  res.stamp = pipelineData_->stamp;

  return true;
}

void
SuturoPerceptionNode::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in)
{
	// init segmentation
	Segmenter *segmenter;
	
	switch (pipelineData_->task_.task_type)
	{
		case suturo_msgs::Task::TASK_1:
		case suturo_msgs::Task::TASK_2:
		case suturo_msgs::Task::TASK_3:
		case suturo_msgs::Task::TASK_4:
		case suturo_msgs::Task::TASK_5:
			logger.logInfo("Using task 4 segmenter");
			segmenter = task4_segmenter_;
		break;
		case suturo_msgs::Task::TASK_6:
			logger.logInfo("Using task 6 segmenter");
			segmenter = task6_segmenter_;
		break;
		default:
			logger.logInfo("Using projection segmenter");
			segmenter = new ProjectionSegmenter();
		break;
	}
	if (!segmenter)
	{
		logger.logError("segmenter creation failed!");
		return;
	}
	
	// start segmentation
	bool segmentation_result = segmenter->segment(cloud_in, pipelineData_, pipelineObjects_);
	
	if (segmentation_result)
	{
		processing_ = false;
	}
	else
	{
		logger.logInfo("segmentation failed");
	}
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projected_points_clusters =
		segmenter->getProjectionClusters();
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projected_point_hulls =
		segmenter->getProjectionClusterHulls();
	for (int i = 0; i < projected_points_clusters.size(); i++)
	{
		std::stringstream ss;
		ss << i;
		ph_.publish_pointcloud(PROJECTED_CLUSTERS_PREFIX_TOPIC + ss.str(), 
				projected_points_clusters[i], DEPTH_FRAME);
	}
	for (int i = 0; i < projected_point_hulls.size(); i++)
	{
		std::stringstream ss;
		ss << i;
		ph_.publish_pointcloud(PROJECTED_CLUSTER_HULLS_PREFIX_TOPIC + ss.str(), 
				projected_point_hulls[i], DEPTH_FRAME);
}
	// Publish the segmentation debug topics
	ph_.publish_pointcloud(TABLE_TOPIC, segmenter->getTablePointCloud()
				, DEPTH_FRAME);

	ph_.publish_pointcloud(DOWNSAMPLED_CLOUD, segmenter->getDownsampledPointCloud()
				, DEPTH_FRAME);
	ph_.publish_pointcloud(POINTS_ABOVE_TABLE_CLOUD, segmenter->getPointsAboveTable()
				, DEPTH_FRAME);

	ph_.publish_pointcloud(PROJECTED_POINTS_TOPIC, segmenter->getProjectedPoints()
				, DEPTH_FRAME);
}

void
SuturoPerceptionNode::receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
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
	cloud_in_ = cloud_in;
  
	// remember time of cloud
  pipelineData_->stamp = inputCloud->header.stamp;

	/*
	logger.logInfo("writing cloud for debugging");
	pcl::PCDWriter writer;
	std::stringstream ss;
	boost::posix_time::ptime t_cloud(boost::posix_time::microsec_clock::local_time());

  // Take the taskname and remove any '/' chars
  std::string taskname_to_write = pipelineData_->task_.task_name;
  boost::algorithm::replace_all(taskname_to_write, "/", "--");
	ss << "/tmp/euroc_c2/cloud-" << taskname_to_write << "-" << boost::posix_time::to_iso_string(t_cloud) << ".pcd";
	logger.logInfo((boost::format("Writing point cloud with %s points to %s") % cloud_in_->points.size() % ss.str()).str());
	writer.writeBinaryCompressed(ss.str(), *cloud_in_);
	logger.logInfo("writing cloud for debugging done");
	*/

	if (task_client_->getTaskDescription().task_type == suturo_msgs::Task::TASK_4 && 
			nodeType_ == GRIPPER)
	{
		// no timeout for gripper segmentation in task 4
		processing_ = false;
	}
	else
	{
		segment(cloud_in);
	}
}
