#ifndef SUTURO_PERCEPTION_POINT_CLOUD_OPERATIONS_H
#define SUTURO_PERCEPTION_POINT_CLOUD_OPERATIONS_H

#include <iostream>

#include <boost/signals2/mutex.hpp>
#include <boost/shared_ptr.hpp>

#include "opencv2/core/core.hpp"

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>

#include "perception_utils/logger.h"
// #include "roi.h"

namespace suturo_perception
{
  /**
   * This class represents methods for general purpose
   * pointcloud processing.
   */
  class PointCloudOperations
  {
    // TODO - PM: Set all Options in one method and use them later?
    // We have to give up static methods then ...
    public:
      PointCloudOperations();

      static void removeNans(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_nanles);
      static void filterZAxis(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
          pcl::PassThrough<pcl::PointXYZRGB> &pass,
          float zAxisFilterMin, float zAxisFilterMax);
      static void downsample(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
                      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, float downsampleLeafSize);
      static void fitPlanarModel(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
            pcl::PointIndices::Ptr inliers, pcl::ModelCoefficients::Ptr coefficients, 
            int planeMaxIterations, 
            double planeDistanceThreshold);
      static void extractInliersFromPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
          pcl::PointIndices::Ptr inliers, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, bool setNegative);
      static bool extractBiggestCluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out, const pcl::PointIndices::Ptr old_inliers,
          pcl::PointIndices::Ptr new_inliers, double ecClusterTolerance,
          int ecMinClusterSize, int ecMaxClusterSize);
			static bool extractBiggestClusters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
				  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &clouds_out, 
					const pcl::PointIndices::Ptr old_inliers, std::vector<pcl::PointIndices::Ptr> &new_inliers_vec,
    double ecClusterTolerance, int ecMinClusterSize, int ecMaxClusterSize);
      static void extractAllPointsAbovePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud, 
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
          pcl::PointIndices::Ptr object_indices, 
          int convex_hull_dimension, double prismZMin, double prismZMax);
      static void extractAllPointsAbovePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, 
          const pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_cloud, 
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out,
          pcl::PointIndices::Ptr object_indices, 
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points,
         int convex_hull_dimension, double prismZMin, double prismZMax);
      static void projectToPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in,
          pcl::PointIndices::Ptr object_indices, pcl::ModelCoefficients::Ptr coefficients,
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out);
  };
}

#endif
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
