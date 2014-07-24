#include "table_from_depth_img_node.h"

TableFromDepthImageNode::TableFromDepthImageNode(ros::NodeHandle &n, std::string depthTopic) : 
  nodeHandle_(nodeHandle), 
  depthTopic_(depthTopic)
{
	logger = perception_utils::Logger("TableFromDepthImageNode");
}

TableFromDepthImageNode::receiveCloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*inputCloud,*cloud_in);
	
	
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "table from pointcloud");
}