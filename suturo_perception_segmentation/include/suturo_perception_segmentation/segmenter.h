#ifndef SUTURO_PERCEPTION_SEGMENTER
#define SUTURO_PERCEPTION_SEGMENTER

#include <perception_utils/pipeline_object.hpp>
#include <perception_utils/pipeline_data.hpp>
#include <pcl/point_types.h>

namespace suturo_perception
{
  class Segmenter
  {
    public:
      Segmenter() {}

      virtual bool segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, PipelineData::Ptr &pipeline_data, PipelineObject::VecPtr &pipeline_objects) = 0;
      // Should be available after a successful segmentation
      virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTablePointCloud() = 0;
      virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getDownsampledPointCloud() = 0;
      virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointsAboveTable() = 0;
      virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getProjectedPoints() = 0;
      virtual std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getProjectionClusters() = 0;
      virtual std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getProjectionClusterHulls() = 0;
  };
}

#endif // SUTURO_PERCEPTION_SEGMENTER
