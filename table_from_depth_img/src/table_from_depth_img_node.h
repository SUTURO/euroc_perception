#ifndef TABLE_FROM_DEPTH_IMAGE_NODE_H
#define TABLE_FROM_DEPTH_IMAGE_NODE_H

#include "ros/ros.h"

#include "perception_utils/logger.h"
#include "suturo_perception_msgs/GetTable.h"

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

class TableFromDepthImageNode
{
  public:
    TableFromDepthImageNode(ros::NodeHandle &nodeHandle, std::string depthTopic);
    bool getTable(suturo_perception_msgs::GetTable::Request &req, suturo_perception_msgs::GetTable::Response &res);

    void receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud);
  private:
    ros::NodeHandle nodeHandle_;
    ros::ServiceServer clusterService;
    std::string cloudTopic_;
		perception_utils::Logger logger;
};

#endif // TABLE_FROM_DEPTH_IMAGE_NODE_H
