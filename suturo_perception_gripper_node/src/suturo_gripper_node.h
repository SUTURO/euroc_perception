#ifndef SUTURO_PERCEPTION_SCENE_NODE_H
#define SUTURO_PERCEPTION_SCENE_NODE_H

#include "ros/ros.h"

#include "perception_utils/logger.h"
#include "perception_utils/pipeline_object.hpp"
#include "perception_utils/pipeline_data.hpp"
#include "suturo_perception_msgs/GetGripper.h"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

class SuturoGripperNode
{
  public:
    SuturoGripperNode(ros::NodeHandle &nodeHandle, std::string imageTopic, std::string depthTopic);
    bool getGripper(suturo_perception_msgs::GetGripper::Request &req, suturo_perception_msgs::GetGripper::Response &res);

    void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
    void clusterFromProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<pcl::PointIndices::Ptr> &original_indices);
  private:
    ros::NodeHandle nodeHandle_;
    ros::ServiceServer clusterService_;
		std::string imageTopic_;
    std::string cloudTopic_;
    suturo_perception::Logger logger;
		int idx_;
    int objidx_;
		bool processing_;

    ros::Publisher markerPublisher_;
    int maxMarkerId_;
    void publish_marker(suturo_perception::PipelineObject::VecPtr &objects);
		
		suturo_perception::PipelineObject::VecPtr pipelineObjects_;
    suturo_perception::PipelineData::Ptr pipelineData_;
		// pcl::ModelCoefficients::Ptr coefficients_; 
    
    
};

#endif // TABLE_FROM_DEPTH_IMAGE_NODE_H
