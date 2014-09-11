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

class CloudProjector
{
public:
  static pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_project(const cv::Mat &depth_image_in, const cv::Mat &rgb_image, const tf::StampedTransform &transform);
};

#endif 