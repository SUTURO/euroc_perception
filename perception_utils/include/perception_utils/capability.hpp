#ifndef CAPABILITY_H
#define CAPABILITY_H

#include "perception_utils/pipeline_object.hpp"
#include <boost/signals2/mutex.hpp>

namespace suturo_perception
{
	class Capability
	{
		public:
			// The PerceivedObject that will be modified by the capability
			Capability(PerceivedObject &obj) : perceivedObject(obj) {};

			/* The execute method that has to be implemented by the deriving classes.
			* This should modify the perceivedObject that was set before.
			*/
			virtual void execute() = 0;

		protected:
			// TODO: Locking when converting to MT
			suturo_perception_lib::PerceivedObject &perceivedObject;
	};
}

#endif
