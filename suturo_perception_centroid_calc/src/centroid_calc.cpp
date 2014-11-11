#include "suturo_perception_centroid_calc/centroid_calc.h"

#include <perception_utils/threadsafe_hull.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using namespace suturo_perception;

CentroidCalc::CentroidCalc(PipelineData::Ptr pipelineData, PipelineObject::Ptr pipelineObject) :
  Capability(pipelineData, pipelineObject)
{
  logger = Logger("suturo_perception_centroid_calc");
}

void CentroidCalc::execute()
{
  // Calculate the volume of each cluster
  // Create a convex hull around the cluster and calculate the total volume
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_points_from_hull (new pcl::PointCloud<pcl::PointXYZRGB> ());

  double volume = ThreadsafeHull::computeConvexHull<pcl::PointXYZRGB>(pipelineObject_->get_pointCloud(), hull_points, true);

  // Centroid calulcation
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid (*hull_points, centroid);  

  logger.logInfo((boost::format("Centroid: %s, %s, %s") % centroid[0] % centroid[1] % centroid[2]).str());

  Point centroid_point;
  centroid_point.x = centroid[0];
  centroid_point.y = centroid[1];
  centroid_point.z = centroid[2];
  pipelineObject_->set_c_centroid(centroid_point);
  pipelineObject_->set_c_volume(volume);
}
