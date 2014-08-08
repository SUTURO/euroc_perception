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
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "perception_utils/logger.h"

class CorrespondenceTransform
{
  private:
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_correspondences_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr observed_correspondences_;
    // Observed points in Vector format
    Eigen::VectorXf y0_;

    // Initial pose guess
    Eigen::Matrix< float, 6, 1 > initial_x_;
    // The (iteratively) estimated pose
    Eigen::Matrix< float, 6, 1 > x_;

    int max_iterations_;
    int iterations_;
    // The pose estimation is done, when 
    // abs( (deltax.norm() / x.norm()) < convergence_threshold_
    // E.g. the change in the pose parameters is small enough ...
    double convergence_threshold_;

  public:

    CorrespondenceTransform()
    {
      // Assume a neutral pose first ...
      initial_x_[0] = initial_x_[1] = initial_x_[2] = initial_x_[3] = initial_x_[4] = initial_x_[5] = 0;
      x_ = initial_x_;
      max_iterations_ = 20;
      iterations_ = 0;
      convergence_threshold_ = 1e-6;
    }
    Eigen::VectorXf getVectorFromPointCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr p);

    Eigen::Matrix4f getRotationMatrixFromPose(Eigen::Matrix< float, 6, 1 > pose);

    // Transform a cloud with a given pose + a small varation of one of the parameters in the pose
    // variation_idx indicates which index should be added by e
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformVariedPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
        Eigen::Matrix< float, 6, 1 > pose, int variation_idx, float e);

    pcl::PointCloud<pcl::PointXYZ>::Ptr generateBoxModelFromPose(Eigen::Matrix< float, 6, 1 > pose);

    // It's necessary to call this function.
    // This method takes two Pointclouds that describe corresponding points.
    // The format is as follows:
    //   - The first point in model_correspondences corresponds to
    //     the first point in observed_correspondences
    //   - The rest is analog ...
    //   - You should have atleast 3 corresponding points for a 6DOF pose estimation
    void setCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr model_correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr observed_correspondences);

    void execute();
    Eigen::Matrix< float, 6, 1 > getEstimatedPose();
    // How many iteration have been done for the pose estimation?
    int getIterationCount();
};

#endif
