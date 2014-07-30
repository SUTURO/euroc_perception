#ifndef DETECTED_PLANE_H
#define DETECTED_PLANE_H

#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/registration/distances.h>

// This class will be used to store all
// the gathered informations for a detected plane.
// The CuboidMatcher will look for atleast 2 planes
// in a given PointCloud and use their normals
// to estimate a Cuboid.
class DetectedPlane
{
  public:
    DetectedPlane();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Get the angle between the normal of *this*
    // ModelCoefficients (e.g. The Normal vector of the detected plane)
    // The output is in RADIAN
    float angleBetween(Eigen::Vector3f v);

    // Trivial setter and getter
    Eigen::Vector3f getCentroid(){ return centroid_; }
    void setCentroid(Eigen::Vector4f v){ 
	centroid_[0] = v[0];
	centroid_[1] = v[1];
	centroid_[2] = v[2];
    }
    Eigen::Vector3f getNormOrigin(){ return normOrigin_; }
    void setNormOrigin(Eigen::Vector3f no){ normOrigin_ = no;}

    pcl::ModelCoefficients::Ptr getCoefficients(){ return coefficients_; } 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  getPoints(){ return points_; }
    pcl::PointIndices::Ptr getInliers(){ return inliers_; }

    Eigen::Vector3f getCoefficientsAsVector3f();
    Eigen::Vector3f getCoefficientsAsPointXYZRGB();
    Eigen::Vector3f getCentroidAsVector3f();
    Eigen::Vector3f getCentroidAsPointXYZ(); // TODO implement and use!

  private:
    // Every plane in 3d can be described with 4 variables
    // Store these 4 variables for later use
    pcl::ModelCoefficients::Ptr coefficients_;
    // The inliers of the RANSAC plane fitting process
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_;
    // The centroid of the plane
    Eigen::Vector3f centroid_;
    // The projected Origin of the normal vector on the object
    // This is just for visualization purposes
    Eigen::Vector3f normOrigin_;
    // The inlier indices for the plane points
    pcl::PointIndices::Ptr inliers_;

};
#endif
