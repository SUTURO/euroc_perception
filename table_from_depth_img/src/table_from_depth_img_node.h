#ifndef TABLE_FROM_DEPTH_IMAGE_NODE_H
#define TABLE_FROM_DEPTH_IMAGE_NODE_H

#include "perception_utils/logger.h"

#include "ros/ros.h"
#include <sensor_msgs/PointCloud.h>

class TableFromDepthImageNode
{
  public:
    TableFromDepthImageNode(ros::NodeHandle &nodeHandle, std::string depthTopic);
    void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
  private:
    ros::NodeHandle nodeHandle_;
    std::string cloudTopic_;
		perception_utils::Logger logger;
};

#endif // TABLE_FROM_DEPTH_IMAGE_NODE_H
