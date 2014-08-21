#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include "perception_utils/basic_io_pcd.h"

class CropBoxIO : public BasicIOPCD
{
  public:
    CropBoxIO(int argc, char** argv) : BasicIOPCD(argc, argv)
    {
    }
    void execute()
    {
      Eigen::Vector4f min(0,0,0,0);
      Eigen::Vector4f max(1,0.5,1,0);
      pcl::CropBox<pcl::PointXYZRGB> c;
      c.setMin(min);
      c.setMax(max);
      c.setInputCloud(input_cloud_);
      c.filter(*output_cloud_);
    }
};

int
main (int argc, char** argv)
{
  CropBoxIO cio(argc,argv);
  cio.execute();
  cio.write_pcd();
  return (0);
}
