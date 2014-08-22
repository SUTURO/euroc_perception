#ifndef CAPABILITY_H
#define CAPABILITY_H

#include "perception_utils/pipeline_object.hpp"
#include <boost/signals2/mutex.hpp>
#include <string>

namespace suturo_perception
{
	class Capability
	{
		public:
			// The PerceivedObject that will be modified by the capability
			Capability() {}
			
			virtual Capability *instantiateCapability(PipelineObject::Ptr obj, bool enabled = true) = 0;
      /*
       : pipelineObject_(obj)
      {
        enabled_ = enabled;
      };
      */

			/* The execute method that has to be implemented by the deriving classes.
			 * This should modify the perceivedObject that was set before.
			 */
			virtual void execute() = 0;
      
      /* The name of the capability. Used to en/disable capabilities in the
       * pipeline.
       */
      virtual std::string getName() = 0;
      
      bool isEnabled() { return enabled_; };
      void setEnabled(bool enabled) { enabled_ = enabled; };

		protected:
			PipelineObject::Ptr pipelineObject_;
      
      bool enabled_;
	};
}

#endif
