#ifndef CAPABILITY_H
#define CAPABILITY_H

#include "perception_utils/pipeline_object.hpp"

#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <suturo_perception_centroid_calc/centroid_calc.h>
#include <suturo_perception_shape_detection/shape_detector.h>

#include <boost/signals2/mutex.hpp>
#include <string>

namespace suturo_perception
{
	class Capability
	{
		public:
      // add your capability BEFORE last_capability!!!
      enum CapabilityType { CUBOID_MATCHER, CENTROID_CALC, SHAPE_DETECTOR, LAST_CAPABILITY };

			// The PerceivedObject that will be modified by the capability
			Capability(PipelineObject::Ptr obj, bool enabled = true) : pipelineObject_(obj)
      {
        enabled_ = enabled;
      }

      static int capabilityCount()
      {
        return LAST_CAPABILITY;
      }

      // TODO: move this into pipeline
      static Capability* instantiateCapability(CapabilityType type, PipelineObject::Ptr obj, bool enabled = true)
      {
        switch (type)
        {
          case CUBOID_MATCHER:
          return new CuboidMatcher(obj);
          break;  

          case CENTROID_CALC:
          return new CentroidCalc(obj);
          break;

          case SHAPE_DETECTOR:
          return new ShapeDetector(obj);
          break;

          default:
          return NULL;
          break;
        }
      }

			/* The execute method that has to be implemented by the deriving classes.
			 * This should modify the perceivedObject that was set before.
			 */
			virtual void execute() = 0;
      
      /* The name of the capability. Used to en/disable capabilities in the
       * pipeline.
       */
      virtual std::string getName() = 0;
      
      bool isEnabled() { return enabled_; }
      void setEnabled(bool enabled) { enabled_ = enabled; }

		protected:
			PipelineObject::Ptr pipelineObject_;
      
      bool enabled_;
	};
}

#endif
