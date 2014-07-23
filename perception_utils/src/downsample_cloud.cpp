#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include "perception_utils/basic_io_pcd.h"

class DownsampleIO : public BasicIOPCD
{
  public:
    DownsampleIO(int argc, char** argv) : BasicIOPCD(argc, argv)
    {
    }
    void execute()
    {
      // Create the filtering object
      pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud (input_cloud_);
      sor.setLeafSize (0.01f, 0.01f, 0.01f);
      sor.filter (*output_cloud_);
    }
};

int
main (int argc, char** argv)
{
  DownsampleIO dio(argc,argv);
  dio.execute();
  dio.write_pcd();
  return (0);
}
