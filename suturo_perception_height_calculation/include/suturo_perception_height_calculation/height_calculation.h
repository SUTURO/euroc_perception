#ifndef SUTURO_PERCEPTION_HEIGHT_CALCULATION
#define SUTURO_PERCEPTION_HEIGHT_CALCULATION

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include "opencv2/core/core.hpp"
#include <boost/signals2/mutex.hpp>

#include <perception_utils/logger.h>
#include <perception_utils/capability.hpp>
#include <perception_utils/pipeline_data.hpp>
#include <perception_utils/pipeline_object.hpp>

namespace suturo_perception
{
  class HeightCalculation : public Capability
  {
    public:
      HeightCalculation(PipelineData::Ptr data, PipelineObject::Ptr obj);

      // capability methods
      void execute();
      std::string getName() { return "height"; }

      double calculateHeight(pcl::ModelCoefficients::Ptr table, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud); 

    private:
      Logger logger;
  };
}
#endif 
