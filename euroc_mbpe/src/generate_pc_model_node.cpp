#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <math.h>
#include <euroc_mbpe/generate_pc_model.h>

int
main (int argc, char** argv)
{
  // vector of filename indices
  std::vector<int> filenames;
  std::string output_filename;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 1)
  {
    std::cout << "Usage: output_file_path.pcd\n";
    exit (-1);
  }
  GeneratePointCloudModel g;

  output_filename = argv[filenames.at(0)];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // output_cloud = generateBox(0.05, 0.05, 0.05, 3000);
  // output_cloud = generateCylinder(0.1, 0.02, 5000);
  output_cloud = g.generateComposed();

  // write pcd
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << output_filename;
  writer.write(ss.str(), *output_cloud);
  std::cerr << "Saved " << output_cloud->points.size () << " data points" << std::endl;

  return (0);
}

