#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include "perception_utils/basic_io_pcd.h"

class GetMinMaxIO : public BasicIOPCD
{
  public:
    GetMinMaxIO(int argc, char** argv) : BasicIOPCD(argc, argv)
    {
    }
    void execute()
    {
      pcl::PointXYZRGB min_pt;
      pcl::PointXYZRGB max_pt;     
      pcl::getMinMax3D(*input_cloud_, min_pt, max_pt);
      output_cloud_->push_back(min_pt);
      output_cloud_->push_back(max_pt);
    }
};

int
main (int argc, char** argv)
{
  GetMinMaxIO mmio(argc,argv);
  mmio.execute();
  mmio.write_pcd();
  return (0);
}
