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
      // These 2 points span a Box. The box can be rotated and translated with setTransform
      // or setRotation
      //
      Eigen::Vector4f min(0,0,0,0);
      Eigen::Vector4f max(1,0.5,1,0);
      pcl::CropBox<pcl::PointXYZRGB> c(true);
      c.setMin(min);
      c.setMax(max);
      c.setInputCloud(input_cloud_);
      std::vector<int> inside_point_indices;
      boost::posix_time::ptime s1 = boost::posix_time::microsec_clock::local_time();
      c.filter(inside_point_indices);
      boost::posix_time::ptime e1 = boost::posix_time::microsec_clock::local_time();

      bool cropped_points = inside_point_indices.size() > 0;
      std::cout << "Cropped points: " << inside_point_indices.size() << std::endl;

      output_cloud_->width = input_cloud_->points.size() - inside_point_indices.size();
      output_cloud_->height = 1;

      int delete_idx = -1;

      // It's faster to delete the last element from the list ... So we reverse everything
      for(int i = input_cloud_->points.size()-1; i >= 0; i--)
      // for(int i=0; i < input_cloud_->points.size(); i++)
      {
        if(cropped_points)
        {
          if(delete_idx < 0)
          {
            delete_idx = inside_point_indices.back();
            inside_point_indices.pop_back();
            cropped_points = inside_point_indices.size() > 0;
          }
          if( i == delete_idx)
          {
            // Remember, that you need to request the next idx to avoid
            delete_idx = -1;
            continue;
          }
        }
        output_cloud_->points.push_back (input_cloud_->points[i]); 
      }
      boost::posix_time::ptime e2 = boost::posix_time::microsec_clock::local_time();
      std::cout << "Output points: " << output_cloud_->points.size() << std::endl;
      boost::posix_time::time_duration d1 = e1 - s1;
      boost::posix_time::time_duration d2 = e2 - e1;
      float diff1 = (float)d1.total_microseconds() / (float)1000;
      float diff2 = (float)d2.total_microseconds() / (float)1000;
      std::cout << "Time for CropHull: " << diff1 << "ms" << std::endl;
      std::cout << "Time for removing the matched Points from the input cloud: " << diff2 << "ms" << std::endl;
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
