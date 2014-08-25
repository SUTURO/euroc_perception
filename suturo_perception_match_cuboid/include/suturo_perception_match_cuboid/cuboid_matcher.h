#ifndef CUBOID_MATCHER_H
#define CUBOID_MATCHER_H

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

#include "boost/date_time/posix_time/posix_time.hpp"
#include <boost/thread.hpp>

#include <perception_utils/threadsafe_hull.h>
#include <perception_utils/cuboid.hpp>
#include <perception_utils/capability.hpp>
#include <suturo_perception_match_cuboid/detected_plane.h>

// Constants for the different operation modes of this class
#define CUBOID_MATCHER_MODE_WITHOUT_COEFFICIENTS 0 
#define CUBOID_MATCHER_MODE_WITH_COEFFICIENTS 1

// This class implements the main functionality of the Cuboid
// Matching process.
// It finds the two dominant planes from a given PointCloud
// and aligns them to the cameras base frame.
// When the cuboid has been aligned, a bounding box will
// be calculated
// By following these transformations backwards, we get the orientation
// and the Cuboid. 
// The algorithm can work in two modes:
//  1) CUBOID_MATCHER_MODE_WITHOUT_COEFFICIENTS :
//    Try to estimate a bounding box around the given pointcloud without any other given
//    parameters. This is useful, if you have just a segmented object without any additional data.
//    Since pointclouds for smaller objects are rather noisy,
//    the algorithm might not be able to find a very good right angle
//    between the different faces of the object.
//  2) CUBOID_MATCHER_MODE_WITH_COEFFICIENTS :
//    This mode works similar to the first mode, but aims at cuboid estimation for objects
//    on a plane (e.g. tables, etc.). Since tables and other big planar surfaces
//    have a more stable surface, RANSAC will be able to find a better
//    plane model describing that surface.
//    In this mode, you can pass pcl::ModelCoefficients to this algorithm.
//    These ModelCoefficients should describe a plane where the object lies on.
//    The algorithm will then try to use this plane to estimate the Pose of
//    the Cuboid in the given PointCloud.
namespace suturo_perception
{
  class CuboidMatcher : public suturo_perception::Capability
  {
    static boost::mutex mx;
    public:
      CuboidMatcher(PipelineData::Ptr pipelineData, PipelineObject::Ptr pipelineObject);
      // Return a pointer to the list of detected planes
      std::vector<DetectedPlane> *getDetectedPlanes();

      // Set the input cloud for the algorithm 
      // (for example, a segmented object)
      void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

      // If you set setSaveIntermediateResults(true), the algorithm
      // will place intermediate results of the Cuboid Matching process
      // in a vector of PointClouds, which is accessible
      // by this method.
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getIntermediateClouds(){ return intermediate_clouds_; } 

      // Set this to true to save every transformed pointcloud
      // which will be generated during the fitting process
      // This is helpful, if you want to visualize the intermediate
      // results to understand the algorithm
      void setSaveIntermediateResults(bool b);

      // Print statistics, like angle between different normals etc.
      void setDebug(bool b);

      // This method method checks the angle
      // between v1 and v2.
      // If it's higher than 90 DEG or M_PI/2, a 180 DEG rotated version
      // of v2 will be returned.
      // This helps during the alignment process, to rotate only
      // by small portions
      // If the angle is below 90 DEG, the unmodified v2
      // will be returned
      static Eigen::Vector3f reduceNormAngle(Eigen::Vector3f v1, Eigen::Vector3f v2);

      // Return the amount of executed transformations to fit the cuboid
      int transformationCount();

      // Execute the algorithm
      // @param c A reference to a cuboid instance, where the fitted cuboid informations
      //          will be stored.
      //
      // @return false, if the algorithm could not fit a cuboid to the pointcloud
      // this can happen, if the object hasnt atleast 2 planar surfaces which can
      // be segmented by RANSAC.
      // true, if the algorithm has succeded.
      //
      bool execute(Cuboid::Ptr c);
      void execute(); 
      std::string getName() { return "cuboid"; }

      bool estimationSuccessful(){ return estimation_succesful_; }

      static void computeCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, Eigen::Vector4f &centroid);

      // If you use CUBOID_MATCHER_MODE_WITH_COEFFICIENTS
      // you MUST supply the coefficients with
      // setTableCoefficients() !
      void setMode(int mode){ mode_ = mode; }

      // Set the table coefficients that will be used in
      // CUBOID_MATCHER_MODE_WITH_COEFFICIENTS mode
      void setTableCoefficients(pcl::ModelCoefficients::Ptr table_coefficients)
      { table_coefficients_ = table_coefficients;}

      // This method will use pcl::getMinxMax3D to estimate a BoundingBox around
      // a given PointCloud.
      // In this algorithm, this method will only be used on the properly aligned
      // pointcloud. It is the last step in the estimation process after different
      // rotations have been used to align the object as good as possible
      // to be parallel to the cameras origin
      // 
      // @returns a PointCloud with the 8 Corner points of the estimated BoundingBox
      void computeCuboidCornersWithMinMax3D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points);

      // Given a set of corner points ( size = 8),
      // calculate the various informations for Cuboid
      // (side lengths, centroid, volume)
      Cuboid::Ptr computeCuboidFromBorderPoints(
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points);

      void computeCuboidFromBorderPoints(
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points, Cuboid::Ptr );

    private:
      // If mode == CUBOID_MATCHER_MODE_WITH_COEFFICIENTS, this
      // integer describes the best matching plane
      // that is perpendicular to the given table coefficients
      int good_matching_plane_;

      Eigen::Vector3f getTableCoefficientsAsVector3f();

      // true, if a cuboid could be matched to the given input cloud
      bool estimation_succesful_;

      // Decide which mode you will run:
      //   Without table coefficients
      //   With table coefficients
      //   The first one is less accurate, but can estimate a bounding box
      //   even when the underlying plane of the object is not known.
      int mode_;

      // Rotate the Vector 'normal_to_rotate' into 'base_normal'
      // Returns a rotation matrix which can be used for the transformation
      // The matrix also includes an empty translation vector
      // The resulting matrix can be used to rotate a set of Points
      // in the given PointCloud.
      //
      // By passing store_transformation=true, the transformation
      // will be saved in the class attribute 'transformations'
      Eigen::Matrix< float, 4, 4 > rotateAroundCrossProductOfNormals(
          Eigen::Vector3f base_normal,
          Eigen::Vector3f normal_to_rotate, bool store_transformation);

      // Remove the translation Vector part from a 4x4 matrix
      // This will yield a matrix that holds only the rotational part.
      Eigen::Matrix< float, 3, 3 > removeTranslationVectorFromMatrix(Eigen::Matrix<float,4,4> m);


      // This class will hold record of all necessary normal
      // vectors during the cuboid estimation process.
      // By calling this method, you will transform every stored
      // normal in transformed_normals_  with the given
      // transformation matrix "transformation"
      // and save them back into the list transformed_normals_.
      void updateTransformedNormals(Eigen::Matrix<float,4,4> transformation);

      // Get the first 3 dimensions from a 4 dimensional vector
      Eigen::Vector3f getVector3fFromVector4f(Eigen::Vector4f vec);


      bool debug;

      // Try to find planes on the given pointcloud
      void segmentPlanes();

      bool save_intermediate_results_;

      pcl::ModelCoefficients::Ptr table_coefficients_;

      // The input cloud.
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;

      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> intermediate_clouds_;

      // A vector of all normal vectors from every detected plane
      // During the execution, the object will be rotated and therefore
      // the orientation of the normal vectors should
      // change too (handled by CuboidMatcher::updateTransformedNormals)
      // The transformed normal vectors will be saved in this attribute.
      std::vector<Eigen::Vector3f> transformed_normals_;

      std::vector<DetectedPlane> detected_planes_;
      // A list with all used transformation
      // matrices 
      std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > transformations_;

      // Input parameter for RANSAC: Distance Threshold
      double ransac_distance_threshold_;
  };
}
#endif
