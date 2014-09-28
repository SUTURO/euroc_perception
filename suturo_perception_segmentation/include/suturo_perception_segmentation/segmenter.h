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
      Segmenter();

      bool segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, PipelineData::Ptr &pipeline_data, PipelineObject::VecPtr &pipeline_objects);
  };
}

#endif // SUTURO_PERCEPTION_SEGMENTER
