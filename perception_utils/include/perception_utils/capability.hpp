#ifndef CAPABILITY_H
#define CAPABILITY_H

#include "perception_utils/pipeline_data.hpp"
#include "perception_utils/pipeline_object.hpp"

#include <boost/signals2/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
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
      
      void pipelineExecute()
			{
				executeStart_ = boost::posix_time::microsec_clock::local_time();
				execute();
				executeEnd_ = boost::posix_time::microsec_clock::local_time();
			}
			
			boost::posix_time::time_duration getExecutionTime()
			{
				return executeEnd_ - executeStart_;
			}
			
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
			
			/* The execute method that has to be implemented by the deriving classes.
			 * This should modify the pipelineObject that was set before.
			 */
			virtual void execute() = 0;
			
			// time logging
			boost::posix_time::ptime executeStart_;
			boost::posix_time::ptime executeEnd_;
	};
}

#endif
