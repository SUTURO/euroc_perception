#ifndef PIPELINE_DATA_H
#define PIPELINE_DATA_H

#include <pcl/point_types.h>

namespace suturo_perception
{
	class PipelineData
	{
		public pcl::ModelCoefficients::Ptr coefficients_; 
	};
}

#endif