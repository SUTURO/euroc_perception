#ifndef SUTURO_PERCEPTION_CENTROID_CALC_H
#define SUTURO_PERCEPTION_CENTROID_CALC_H

#include <perception_utils/pipeline_object.hpp>
#include <perception_utils/capability.hpp>
#include <perception_utils/logger.h>

namespace suturo_perception
{
  class CentroidCalc : public Capability
  {
  public:
    CentroidCalc(PipelineData::Ptr pipelineData, PipelineObject::Ptr pipelineObject);
    
    void execute();
    std::string getName() { return "centroid"; }
  protected:
    Logger logger;
  };
}
#endif
