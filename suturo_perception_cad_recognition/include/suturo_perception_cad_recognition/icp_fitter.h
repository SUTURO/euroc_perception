#ifndef SUTURO_PERCEPTION_ICP_FITTER
#define SUTURO_PERCEPTION_ICP_FITTER

/*
 * This node is part of a pipeline
 * to align a CAD model to a (partial) pointcloud
 * of a segmented object. Right now, this class is limited
 * to estimate the Pose of a "Mondamin Pancake Mix". A CAD model of it
 * can be found in this package under test_data/pancake_mix.stl.
 *
 * The CAD model has to be subsampled as a Pointcloud and be
 * passed to this node.
 * You can subsample a CAD model with CloudCompare (http://www.danielgm.net/cc/)
 * Future releases of this software may automate this step.
 *
 * The estimation will be done as follows:
 *   1) Estimate the rough orientation of the partial pointcloud and transform it in a way, that it is standing straight up in the origin
 *   2) Run ICP, to get a refined Pose
 *   3) Calculate the original orientation of the object, by calculating the inverse of every
 *      necessary transformation that has been done.
 */

#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <boost/program_options.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
// #include <perception_utils.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/shot.h>
#include <suturo_perception_match_cuboid/cuboid_matcher.h>

using namespace boost;
using namespace std;

class ICPFitter
{

  #define NO_ICP_MAX_ITERATIONS -1
  #define NO_ICP_DISTANCE -1
  protected:
    Eigen::Vector4f _table_normal;

  public:
    // Attributes
    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _model_cloud;
    // Intermediate pointclouds that have been generated in the execution step
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_model;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_object;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_object_s1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_object_s2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _upwards_object_s3;
    int _max_icp_iterations;
    double _max_icp_distance;
    double _icp_fitness_score;
    // The transformation that has been done by ICP after the initial alignment
    Eigen::Matrix<float, 4, 4> _icp_transform;
    // The inverse of the last transformation
    Eigen::Matrix<float, 4, 4> _icp_transform_inverse;

    std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > rotations_;
    std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > translations_;

    ICPFitter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud, Eigen::Vector4f table_normal) : _cloud_in(cloud_in), _model_cloud(model_cloud), _table_normal(table_normal)
    {
      _max_icp_iterations = NO_ICP_MAX_ITERATIONS;
      _icp_fitness_score = 99; // Default
      _max_icp_distance = NO_ICP_DISTANCE;
    }
    // Eigen::Matrix<float, 4, 4>  getTransformation(); // Available after execution

    // Methods
    Eigen::Matrix< float, 4, 4 > rotateAroundCrossProductOfNormals(
        Eigen::Vector3f base_normal,
        Eigen::Vector3f normal_to_rotate,
        bool store_transformation=false);

    // Calculate a Cuboid object from a given set of corner points. These will be most likely
    // generated by computeCuboidCornersWithMinMax3D
    Cuboid computeCuboidFromBorderPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points);

    // Calculate the corner points of a given point cloud by using pcl::getMinMax3D
    void computeCuboidCornersWithMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points);

    // Return a Translation matrix that will be constructed from the
    // given parameters
    Eigen::Matrix< float, 4, 4> getTranslationMatrix(
        float x, float y, float z);

    // Execute the pancake pose estimation process
    pcl::PointCloud<pcl::PointXYZ>::Ptr execute();

    // Get the final rotation of the object. The rotation will be calculated by
    // multiply every rotation that has been done in the executation step
    Eigen::Matrix<float, 4, 4>  getRotation();

    // Get the final position of the object. The translation will be calculated by
    // adding every translation that has been done in the executation step
    Eigen::Matrix<float, 4, 4>  getTranslation();
    // Remove the Translation vector from a 4x4 matrix. The translation 
    // vector should be in the most-right column.
    Eigen::Matrix< float, 3, 3 > removeTranslationVectorFromMatrix(Eigen::Matrix<float,4,4> m);

    // Get the orientation of the aligned object.
    Eigen::Quaternionf getOrientation(); 

    // Get the origin of the aligned object.
    pcl::PointXYZ getOrigin(); 

    // Set the max iteration count for the final ICP procress. 
    // The default is -1 and means, that the ICP defaults will be used
    void setMaxICPIterations(int v);
  
    // Get the fitness score that has been yielded by the ICP process. This indicates how well a cloud has been aligned to a given model. This value should be < 1e-4 for a proper match, but this depends on your usecase
    double getFitnessScore();

    void setMaxCorrespondenceDistance(double v);
  
};
#endif

