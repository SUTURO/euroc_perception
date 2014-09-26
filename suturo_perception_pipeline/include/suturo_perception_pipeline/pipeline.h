#ifndef SUTURO_PERCEPTION_PIPELINE
#define SUTURO_PERCEPTION_PIPELINE

#include <perception_utils/capability.hpp>
#include <perception_utils/pipeline_object.hpp>
#include <perception_utils/pipeline_data.hpp>

#include <string>
#include <vector>

namespace suturo_perception
{
  class Pipeline
  {
    public:
      // add your capability BEFORE last_capability!!!
      enum CapabilityType 
      { 
        CUBOID_MATCHER, 
        CENTROID_CALC, 
        SHAPE_DETECTOR, 
        COLOR_ANALYSIS,
        MODEL_POSE,
        HEIGHT_CALCULATION,
        
        LAST_CAPABILITY 
      };
      
      Pipeline();
    
      static void execute(PipelineData::Ptr pipeline_data, PipelineObject::VecPtr pipeline_objects);
    protected:
      static Capability* instantiateCapability(CapabilityType type, PipelineData::Ptr data, PipelineObject::Ptr obj, bool enabled = true);
      /**
       * @return -1: default value, 0: disabled, 1: enabled
       */
      static int capabilityEnabled(std::string capability_settings, std::string capability_name);
     
      static std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
      static std::vector<std::string> split(const std::string &s, char delim);
      static std::vector<std::string> split(const std::string &s, const std::string &delims);
  };
}
#endif
