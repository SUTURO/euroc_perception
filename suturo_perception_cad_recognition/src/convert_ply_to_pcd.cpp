#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>

int main(int argc, char** argv){
  std::vector<int> pcd_filenames;
  std::vector<int> ply_filenames;
  pcd_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  ply_filenames = pcl::console::parse_file_extension_argument (argc, argv, ".ply");
  if (pcd_filenames.size () != 1 || ply_filenames.size () != 1)
  {
    std::cout << "Usage: input_file_path.ply output_file_path.pcd\n";
    exit (-1);
  }
  std::string input_filename = argv[ply_filenames.at(0)];
  std::string output_filename = argv[pcd_filenames.at(0)];

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  
  pcl::PLYReader reader;
  if (reader.read(input_filename, *input_cloud) < -1) //* load the file
  // if (pcl::io::loadPLYFile<pcl::PointXYZRGB> (input_filename, *input_cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input PLY file\n");
    exit (-1);
  }
  std::cout << "Loaded "
    << input_cloud->width * input_cloud->height
    << " data points from input pcd" << std::endl;

  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << output_filename;
  writer.write(ss.str(), *input_cloud);
  std::cerr << "Saved " << input_cloud->points.size () << " data points" << std::endl;
}

