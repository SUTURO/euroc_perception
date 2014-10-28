#ifndef IA_CENTROID_H
#define IA_CENTROID_H
#include <suturo_perception_cad_recognition/IA_method.h>

namespace suturo_perception{
  class IACentroid : public IAMethod
  {
    public:
      IACentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud, Eigen::Vector4f table_normal) : suturo_perception::IAMethod(cloud_in,model_cloud,table_normal)
    {

    }
      
    void execute();

    Eigen::Matrix<float, 4, 4> getOrientation();
    pcl::PointXYZ getOrigin();
  };
}
#endif
