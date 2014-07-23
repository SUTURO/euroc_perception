#include "table_from_depth_img_node.h"

TableFromDepthImageNode::TableFromDepthImageNode(ros::NodeHandle &n, std::string depthTopic) : 
  nodeHandle_(nodeHandle), 
  depthTopic_(depthTopic)
{

}
