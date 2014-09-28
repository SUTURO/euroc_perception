#ifndef CAPABILITY_H
#define CAPABILITY_H

#include "perception_utils/pipeline_data.hpp"
#include "perception_utils/pipeline_object.hpp"

#include <boost/signals2/mutex.hpp>
#include <string>

namespace suturo_perception
{
	class Capability
	{
		public:
			// The PerceivedObject that will be modified by the capability
			Capability(PipelineData::Ptr data, PipelineObject::Ptr obj, bool enabled = true) 
      : pipelineData_(data), pipelineObject_(obj), enabled_(enabled)
      {
      }

			/* The execute method that has to be implemented by the deriving classes.
			 * This should modify the pipelineObject that was set before.
			 */
			virtual void execute() = 0;
      
      /* The name of the capability. Used to en/disable capabilities in the
       * pipeline.
       */
      virtual std::string getName() = 0;
      
      bool isEnabled() { return enabled_; }
      void setEnabled(bool enabled) { enabled_ = enabled; }

		protected:
      PipelineData::Ptr pipelineData_;
			PipelineObject::Ptr pipelineObject_;
      
      bool enabled_;
	};
}

#endif
