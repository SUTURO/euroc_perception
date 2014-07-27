#include "perception_utils/publisher_helper.h"

using namespace perception_utils;

PublisherHelper::PublisherHelper(ros::NodeHandle &nh) : _node_handle(nh), _queue_size(DEFAULT_QUEUE_SIZE)
{
  logger = Logger("publisher_helper");
}

ros::Publisher* PublisherHelper::getPublisher(std::string topic)
{
  if(isAdvertised(topic))
  {
    return &_topic_to_publisher_map[topic];
  }
  else
  {
    return NULL;
  }

}

bool PublisherHelper::isAdvertised(std::string topic)
{
  if(_is_advertised_map.find(topic) == _is_advertised_map.end())
    return false; // Element has not been found -> Topic not advertised yet

  return _is_advertised_map[topic];
}

void PublisherHelper::setAdvertised(std::string topic)
{
  _is_advertised_map[topic] = true;
}

void PublisherHelper::publish_pointcloud(ros::Publisher &publisher, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_publish, std::string frame)
{

  if(cloud_to_publish != NULL)
  {
    sensor_msgs::PointCloud2 pub_message;
    pcl::toROSMsg(*cloud_to_publish, pub_message );
    pub_message.header.frame_id = frame;
    publisher.publish(pub_message);
  }
  else
  {
    ROS_ERROR("publish_pointcloud : Input cloud is NULL"); // use ROS_ERROR here because method is static
  }
}

bool PublisherHelper::publish_pointcloud(std::string topic, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_publish, std::string frame)
{
  if( !isAdvertised(topic))
  {
    logger.logError("publish_pointcloud : Given topic is not advertised");
    return false;
  }

  if(cloud_to_publish != NULL)
  {
    sensor_msgs::PointCloud2 pub_message;
    pcl::toROSMsg(*cloud_to_publish, pub_message );
    pub_message.header.frame_id = frame;
    getPublisher(topic)->publish(pub_message);
    return true;
  }
  else
  {
    logger.logError("publish_pointcloud : Input cloud is NULL");
    return false;
  }
}
bool PublisherHelper::publish_cv_mat(std::string topic, cv::Mat &img, std::string frame)
{
  ros::Time time = ros::Time::now();
  std::string image_encoding = "bgr8";
  return publish_cv_mat(topic, img, time , frame, image_encoding);
}

bool PublisherHelper::publish_cv_mat(std::string topic, cv::Mat &img, ros::Time time, std::string frame, std::string image_encoding)
{
  if( !isAdvertised(topic))
  {
    logger.logError("publish_cv_mat : Given topic is not advertised");
    return false;
  }

  cv_bridge::CvImage cv_img;
  cv_img.header.stamp = time;
  cv_img.header.frame_id = frame;
  cv_img.encoding = image_encoding;
  cv_img.image = img;
  getPublisher(topic)->publish(cv_img.toImageMsg());

  return true;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
