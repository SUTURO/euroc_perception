#ifndef SUTURO_PERCEPTION_PROJECTION_SEGMENTER
#define SUTURO_PERCEPTION_PROJECTION_SEGMENTER

#include "suturo_perception_segmentation/segmenter.h"
#include <perception_utils/pipeline_data.hpp>
#include <perception_utils/pipeline_object.hpp>
#include <perception_utils/logger.h>
#include <pcl/point_types.h>

namespace suturo_perception
{
  class ProjectionSegmenter : Segmenter
  {
    public:
      ProjectionSegmenter() : Segmenter()
      {
        logger = Logger("projection_segmenter");
      }

      bool segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, PipelineData::Ptr &pipeline_data, PipelineObject::VecPtr &pipeline_objects);

    protected:
      bool clusterFromProjection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_clusters, pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud, std::vector<int> *removed_indices_filtered, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &extracted_objects, std::vector<pcl::PointIndices::Ptr> &original_indices, PipelineData::Ptr &pipeline_data);

    private:
      Logger logger;
  };
}
#endif // SUTURO_PERCEPTION_PROJECTION_SEGMENTER
