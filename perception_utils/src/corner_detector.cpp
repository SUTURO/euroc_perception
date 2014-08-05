#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include "perception_utils/basic_io_pcd.h"
#include <pcl/keypoints/harris_3d.h>

class CornerIO : public BasicIOPCD
{
  public:
    CornerIO(int argc, char** argv) : BasicIOPCD(argc, argv)
    {
    }
    void execute()
    {

      pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>* harris3D = new 
        pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
      harris3D->setNonMaxSupression(true); 
      harris3D->setRadius (0.08); 
      harris3D->setRadiusSearch (0.08); 
      harris3D->setThreshold(0.10);
      harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::CURVATURE); 
      harris3D->setInputCloud(input_cloud_); 
      pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints_temp (new pcl::PointCloud<pcl::PointXYZI>); 
      pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints (new pcl::PointCloud<pcl::PointXYZI>); 
      harris3D->compute(*keypoints_temp); 
      keypoints = keypoints_temp; 

      pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>); 


      copyPointCloud (*keypoints , *output_cloud_); 
      std::cout << "Computed " << keypoints_ptr->points.size () << " Harris Keypoints  " << std:: endl; 
    }
};

int
main (int argc, char** argv)
{
  CornerIO cio(argc,argv);
  cio.execute();
  cio.write_pcd();
  return (0);
}
