#ifndef SUTURO_PERCEPTION_NODE_H
#define SUTURO_PERCEPTION_NODE_H

#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>

#include "perception_utils/logger.h"
#include "perception_utils/pipeline_object.hpp"
#include "perception_utils/pipeline_data.hpp"
#include "perception_utils/publisher_helper.h"
#include "suturo_perception_msgs/GetGripper.h"
#include <suturo_perception_msgs/PerceptionNodeStatus.h>
#include <perception_utils/node_status.hpp>
#include "suturo_perception_node/SuturoPerceptionConfig.h"
#include <perception_utils/get_euroc_task_description.h>
#include <suturo_perception_segmentation/segmenter.h>
#include <suturo_perception_segmentation/projection_segmenter.h>
#include <suturo_perception_segmentation/task6_segmenter.h>
#include <suturo_perception_segmentation/task4_segmenter.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

class SuturoPerceptionNode
{
  public:
    enum NodeType { SCENE, GRIPPER };

    SuturoPerceptionNode(ros::NodeHandle &nodeHandle, std::string imageTopic, std::string depthTopic, SuturoPerceptionNode::NodeType nodeType);
    void reconfigureCallback(suturo_perception_node::SuturoPerceptionConfig &config, uint32_t level);
    bool getGripper(suturo_perception_msgs::GetGripper::Request &req, suturo_perception_msgs::GetGripper::Response &res);

    void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
		void segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
  private:
    std::string OBJECT_CLOUD_PREFIX_TOPIC;
    std::string TABLE_TOPIC;
    std::string DOWNSAMPLED_CLOUD;
    std::string POINTS_ABOVE_TABLE_CLOUD;
    std::string PROJECTED_POINTS_TOPIC;
    std::string PROJECTED_CLUSTERS_PREFIX_TOPIC;
    std::string PROJECTED_CLUSTER_HULLS_PREFIX_TOPIC;
    std::string LOGGER_NAME;
    std::string SERVICE_NAME;
    std::string MARKER_TOPIC;
    std::string DEPTH_FRAME;

    NodeType nodeType_; // true = gripper, false = scene
    boost::shared_ptr<suturo_perception::NodeStatus> node_status;

    ros::NodeHandle nodeHandle_;
    ros::ServiceServer clusterService_;
		std::string imageTopic_;
    std::string cloudTopic_;
    suturo_perception::Logger logger;
		int idx_;
    int objidx_;
		bool processing_;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in_;
    suturo_perception::PublisherHelper ph_;
		suturo_perception::EurocTaskClient *task_client_;
		suturo_perception::Task6Segmenter *task6_segmenter_;
		suturo_perception::Task4Segmenter *task4_segmenter_;

    ros::Publisher markerPublisher_;
    int maxMarkerId_;
    void publish_marker(suturo_perception::PipelineObject::VecPtr &objects);
		
		suturo_perception::PipelineObject::VecPtr pipelineObjects_;
    suturo_perception::PipelineData::Ptr pipelineData_;
    
    // dynamic reconfigure
    dynamic_reconfigure::Server<suturo_perception_node::SuturoPerceptionConfig> reconfSrv;
    dynamic_reconfigure::Server<suturo_perception_node::SuturoPerceptionConfig>::CallbackType reconfCb;
};

#endif // SUTURO_PERCEPTION_NODE_H
