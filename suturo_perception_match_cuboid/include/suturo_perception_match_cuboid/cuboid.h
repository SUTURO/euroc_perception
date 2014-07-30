#ifndef CUBOID_H
#define CUBOID_H

// This cuboid will be defined by its three edge lengths.
class Cuboid
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Cuboid()
    {
      corner_points = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    float length1;
    float length2;
    float length3;
    Eigen::Vector3f center;
    float volume;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points;
    Eigen::Quaternion<float> orientation;
};
#endif
