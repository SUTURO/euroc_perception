#ifndef TABLE_FROM_DEPTH_IMAGE_NODE_H
#define TABLE_FROM_DEPTH_IMAGE_NODE_H

#include "ros/ros.h"
#include <sensor_msgs/Image.h>

class TableFromDepthImageNode
{
  public:
    TableFromDepthImageNode(ros::NodeHandle &nodeHandle, std::string depthTopic);
    void receiveImage(const semsor_msgs::Image);
  private:
    ros::NodeHandle nodeHandle_;
    std::string depthTopic_;
};

#endif // TABLE_FROM_DEPTH_IMAGE_NODE_H
