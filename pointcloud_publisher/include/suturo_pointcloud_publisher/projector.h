#ifndef CLOUD_PROJECTOR_H
#define CLOUD_PROJECTOR_H

#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <suturo_msgs/Task.h>

namespace suturo_perception {
  typedef struct ROI_
  {
    int x;
    int y;
    int w;
    int h;
  } ROI;
  
  class CloudProjector
  {
  public:
    static bool getTransform(const ros::NodeHandle &node, const std::string &frame_from, const std::string &frame_to, tf::StampedTransform &transform_);
    static void printTransform(const tf::StampedTransform &transform);
    
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr depthProject(const cv::Mat &depth_image_in, const cv::Mat &rgb_image, const tf::StampedTransform &transform, const bool projectColors);
    static ROI getCloudROI(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, const tf::StampedTransform &transform);
  };
}

#endif 
