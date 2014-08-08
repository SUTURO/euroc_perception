#ifndef CORRESPONDENCE_TRANSFORM_H
#define CORRESPONDENCE_TRANSFORM_H

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <boost/program_options.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "perception_utils/logger.h"

class CorrespondenceTransform
{
  public:

  CorrespondenceTransform()
  {
    // Empty constructor
  }
  Eigen::VectorXf getVectorFromPointCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr p);

  Eigen::Matrix4f getRotationMatrixFromPose(Eigen::Matrix< float, 6, 1 > pose);

  // Transform a cloud with a given pose + a small varation of one of the parameters in the pose
  // variation_idx indicates which index should be added by e
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformVariedPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      Eigen::Matrix< float, 6, 1 > pose, int variation_idx, float e);

  pcl::PointCloud<pcl::PointXYZ>::Ptr generateBoxModelFromPose(Eigen::Matrix< float, 6, 1 > pose);
};

#endif
