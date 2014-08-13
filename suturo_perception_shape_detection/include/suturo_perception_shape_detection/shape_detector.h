#ifndef SUTURO_PERCEPTION_SHAPE_DETECTION_H
#define SUTURO_PERCEPTION_SHAPE_DETECTION_H

#include <perception_utils/logger.h>
#include <perception_utils/capability.hpp>
#include <perception_utils/pipeline_object.hpp>
#include <perception_utils/shape.hpp>

#include <pcl/point_types.h>

namespace suturo_perception
{
  class ShapeDetector : public Capability
  {
    public:
      ShapeDetector(PipelineObject::Ptr obj);
      void detectShape(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudIn);
      Shape getShape();

      // capability method
      void execute();
    private:
      Shape shape;
      Logger logger;
  };
}
#endif


