#include "perception_utils/publisher_helper.h"

using namespace suturo_perception;

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

void PublisherHelper::publish_marker(PipelineObject::VecPtr &objects, std::string frame, ros::Publisher markerPublisher, int *maxMarkerId)
{
  for (int i = 0; i < *maxMarkerId; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = "suturo_perception";
    marker.id = i;
    marker.action = visualization_msgs::Marker::DELETE;
    markerPublisher.publish(marker);
  }

  // Delete cuboids
  // for (int i = 0; i < *maxMarkerId; i++)
  // {
  //   visualization_msgs::Marker marker;
  //   marker.header.frame_id = frame;
  //   marker.header.stamp = ros::Time();
  //   marker.ns = "suturo_perception";
  //   marker.id = i+100;
  //   marker.action = visualization_msgs::Marker::DELETE;
  //   markerPublisher.publish(marker);
  // }

  for (int i = 0; i < objects.size(); i++)
  {
    PipelineObject::Ptr obj = objects[i];
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time();
    marker.ns = "suturo_perception_centroids";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obj->get_c_centroid().x;
    marker.pose.position.y = obj->get_c_centroid().y;
    marker.pose.position.z = obj->get_c_centroid().z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markerPublisher.publish(marker);

    // Publish visualization markers
    visualization_msgs::Marker cuboidMarker;
    cuboidMarker.header.frame_id = frame;
    cuboidMarker.header.stamp = ros::Time();
    cuboidMarker.ns = "suturo_perception_cuboids";
    cuboidMarker.id = i + 100;
    cuboidMarker.lifetime = ros::Duration(10);
    cuboidMarker.type = visualization_msgs::Marker::CUBE;
    cuboidMarker.action = visualization_msgs::Marker::ADD;

    cuboidMarker.pose.position.x = obj->get_c_cuboid()->center[0];
    cuboidMarker.pose.position.y = obj->get_c_cuboid()->center[1];
    cuboidMarker.pose.position.z = obj->get_c_cuboid()->center[2];
    cuboidMarker.pose.orientation.x = obj->get_c_cuboid()->orientation.x();
    cuboidMarker.pose.orientation.y = obj->get_c_cuboid()->orientation.y();
    cuboidMarker.pose.orientation.z = obj->get_c_cuboid()->orientation.z();
    cuboidMarker.pose.orientation.w = obj->get_c_cuboid()->orientation.w();
    cuboidMarker.scale.x = obj->get_c_cuboid()->length1;
    cuboidMarker.scale.y = obj->get_c_cuboid()->length2;
    cuboidMarker.scale.z = obj->get_c_cuboid()->length3;
    cuboidMarker.color.a = 1.0;
    cuboidMarker.color.r = 1.0;
    cuboidMarker.color.g = 0.0;
    cuboidMarker.color.b = 0.0;
    if(obj->get_c_cuboid_success())
    {
      markerPublisher.publish(cuboidMarker);
    }
    else
    {
      std::cout << "Invalid cuboid. Did not publish it" << std::endl;
    }

  }

  *maxMarkerId = objects.size();
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
