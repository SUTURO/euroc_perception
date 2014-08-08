#ifndef SUTURO_PERCEPTION_PIPELINE
#define SUTURO_PERCEPTION_PIPELINE

#include <perception_utils/pipeline_object.hpp>
#include <perception_utils/pipeline_data.hpp>

namespace suturo_perception
{
  class Pipeline
  {
    public:
      Pipeline();
    
      static void execute(PipelineData::Ptr pipeline_data, PipelineObject::VecPtr pipeline_objects);
  };
}
#endif