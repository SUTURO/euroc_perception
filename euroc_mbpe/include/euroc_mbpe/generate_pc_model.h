#ifndef _GENERATE_PC_MODEL_H
#define _GENERATE_PC_MODEL_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <math.h>
#include <suturo_msgs/Task.h>

class GeneratePointCloudModel
{
  private:
    Eigen::Matrix4f getRotationMatrixFromPose(Eigen::Matrix< float, 6, 1 > pose);
  public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateBox(double size_x, double size_y,
        double size_z, int total_points);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateCylinder(double length, double radius, int total_points);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateComposed();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateComposed(std::vector<suturo_msgs::Shape>  &shapes);
};
#endif
