#include "suturo_perception_height_calculation/height_calculation.h"

using namespace suturo_perception;

HeightCalculation::HeightCalculation(PipelineData::Ptr data, PipelineObject::Ptr obj) : Capability(data, obj)
{
  logger = Logger("height_calculation");
}

void
HeightCalculation::execute()
{
  pcl::ModelCoefficients::Ptr table = pipelineData_->coefficients_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pipelineObject_->get_pointCloud();
  double height = calculateHeight(table, cloud);
  pipelineObject_->set_c_height(height);
}


double HeightCalculation::calculateHeight(pcl::ModelCoefficients::Ptr table, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  if (table->values.size() != 4)
  {
    logger.logError("table doesn't have 4 coefficients!");
    return -1.0;
  }
  
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  // plane coefficients
  float a,b,c,d;
  a = table->values[0];
  b = table->values[1];
  c = table->values[2];
  d = table->values[3];

  // divisor
  float e;
  e = sqrt(a*a + b*b + c*c);

  // result
  double height = -1.0;

  for (int i = 0; i < cloud->points.size(); i++)
  {
    pcl::PointXYZRGB *p = &cloud->points[i];
    double tmp = ( a * p->x + b * p->y + c * p->z + d ) / e;
    tmp = tmp < 0 ? -tmp : tmp; // abs
    if (tmp > height)
      height = tmp;
  }
  e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "height calculation");

  return height;
} 
