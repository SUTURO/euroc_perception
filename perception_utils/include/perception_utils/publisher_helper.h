#ifndef PUBLISHER_HELPER_H
#define PUBLISHER_HELPER_H

#include "ros/ros.h"
#include "perception_utils/logger.h"
#include "perception_utils/pipeline_object.hpp"

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <visualization_msgs/Marker.h>

#include "opencv2/core/core.hpp"
#include <cv_bridge/cv_bridge.h>

#include <string>
#include <map>
#include <unistd.h>

#define DEFAULT_QUEUE_SIZE 5

namespace suturo_perception
{
  class PublisherHelper
  {
    private:
      std::map<std::string,ros::Publisher> _topic_to_publisher_map;
      std::map<std::string,bool> _is_advertised_map;
      ros::NodeHandle &_node_handle;
      int _queue_size;
      Logger logger;

      void setAdvertised(std::string topic);

    public:
      // Constructor
      PublisherHelper(ros::NodeHandle& nh);
      /**
       * Is the given Topic already advertised by PublisherHelper::advertise?
       */
      bool isAdvertised(std::string topic);

      /**
       * Returns the publisher object for a given Topic.
       * If no publisher is existing for the topic, NULL will be returned.
       */
      ros::Publisher *getPublisher(std::string topic);

      /**
       * Advertise a topic with Type T at the ROS Master
       * The publisher instance will be stored in an internal hashmap, which can 
       * can then be accessed with the getPublisher() Method
       */
      template<class T>
      void advertise(std::string topic)
      {
				logger.logInfo((boost::format("advertising topic %s") % topic).str());
        if(!isAdvertised(topic))
        {
          _topic_to_publisher_map[topic] = _node_handle.advertise<T> (topic, _queue_size);
          // std::cout << _topic_to_publisher_map[topic].getTopic() << std::endl;
          setAdvertised(topic);
          // sleep(10);
        }
        else
        {
          logger.logError("Tried to advertise on an already existing topic");
        }
        logger.logInfo((boost::format("isAdvertised(%s) == %s") % topic % isAdvertised(topic)).str());
      }

      /**
       * Publish a pointcloud to a given publisher. This method does NOT use
       * the Publisher Hashmap provided by this class! You can use it to publish
       * a PCL Pointcloud to an already advertised topic and a given ros::Publisher.
       */
      static void publish_pointcloud(ros::Publisher &publisher, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_publish, std::string frame);

      /**
       * Publish a pointcloud by passing the name of topic.
       * This method will try to find a publisher for the given topic, and post the
       * passed PointCLoud with the Publisher.
       *
       * @return false, if no publisher is advertised for the given topic name
       *         false, if the input cloud is empty
       *         true otherwise
       */
      bool publish_pointcloud(std::string topic, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_publish, std::string frame);

      /**
       * Publish a given cv::Mat with ros::Time::now() as Timestamp
       * and bgr8 as image encoding
       *
       * @return False, when the given topic is not advertised. True otherwise.
       */
      bool publish_cv_mat(std::string topic, cv::Mat &img, std::string frame);

      /**
       * Publish a given cv::Mat with the passed options
       */
      bool publish_cv_mat(std::string topic, cv::Mat &img, ros::Time time, std::string frame, std::string image_encoding);

      
      static void publish_marker(PipelineObject::VecPtr &objects, std::string frame, ros::Publisher markerPublisher, int *maxMarkerId);
  };
}

#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
