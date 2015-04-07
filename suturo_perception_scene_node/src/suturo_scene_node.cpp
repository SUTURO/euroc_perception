#include "suturo_scene_node.h"

#include <perception_utils/point_cloud_operations.h>
#include <perception_utils/publisher_helper.h>
#include <suturo_perception_segmentation/projection_segmenter.h>
#include <suturo_perception_pipeline/pipeline.h>
#include <suturo_perception_msgs/EurocObject.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace suturo_perception;

SuturoSceneNode::SuturoSceneNode(ros::NodeHandle &n, std::string imageTopic, std::string depthTopic) : 
  nodeHandle_(n), 
  imageTopic_(imageTopic),
  cloudTopic_(depthTopic)
{
	logger = Logger("SuturoPerceptionSceneNode");
  clusterService_ = nodeHandle_.advertiseService("/suturo/perception/GetScene", 
    &SuturoSceneNode::getScene, this);
	idx_ = 0;
  objidx_ = 0;

  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/suturo/perception/cuboid_markers", 0);
  maxMarkerId_ = 0;

}

bool
SuturoSceneNode::getScene(suturo_perception_msgs::GetCameraPerception::Request &req, suturo_perception_msgs::GetCameraPerception::Response &res)
{
	res.id = idx_;
	idx_++;

  pipelineData_ = PipelineData::Ptr(new PipelineData());
	
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
    euObj.frame_id = "/sdepth";
    euObj.c_id = objidx_;
    objidx_++;
    res.objects.push_back(euObj);
  }

  logger.logInfo("results sent, publishing markers");
  PublisherHelper::publish_marker(pipelineObjects_, "/sdepth", markerPublisher_, &maxMarkerId_);

  return true;
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
  
  
  ProjectionSegmenter projection_segmenter;
  if (!projection_segmenter.segment(cloud_in, pipelineData_, pipelineObjects_))
  {
    return;
  }

	processing_ = false;
}
