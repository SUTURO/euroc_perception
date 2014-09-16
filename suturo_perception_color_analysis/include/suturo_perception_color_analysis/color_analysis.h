#ifndef SUTURO_PERCEPTION_COLOR_ANALYSIS
#define SUTURO_PERCEPTION_COLOR_ANALYSIS

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
  typedef struct HSVColor_ {
    uint32_t h;
    double s;
    double v;
  } HSVColor;

  class ColorAnalysis : public Capability
  {
    public:
      ColorAnalysis(PipelineData::Ptr data, PipelineObject::Ptr obj);

      HSVColor getAverageColorHSV(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in);
      HSVColor convertRGBToHSV(uint32_t rgb);
      uint32_t convertHSVToRGB(HSVColor hsv);

      void setLowerSThreshold(double t) { s_lower_threshold = t; };
      void setUpperSThreshold(double t) { s_upper_threshold = t; };
      void setLowerVThreshold(double t) { v_lower_threshold = t; };
      void setUpperVThreshold(double t) { v_upper_threshold = t; };

      double getLowerSThreshold() { return s_lower_threshold; };
      double getUpperSThreshold() { return s_upper_threshold; };
      double getLowerVThreshold() { return v_lower_threshold; };
      double getUpperVThreshold() { return v_upper_threshold; };

      // capability methods
      void execute();
      std::string getName() { return "color"; }

    private:
      bool inHSVThreshold(HSVColor col);

      Logger logger;
      
      double s_lower_threshold;
      double s_upper_threshold;
      double v_lower_threshold;
      double v_upper_threshold;
  };
}
#endif 
