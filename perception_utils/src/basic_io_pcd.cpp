#include "perception_utils/basic_io_pcd.h"

BasicIOPCD::BasicIOPCD(int argc, char** argv)
{
  // vector of filename indices
  std::vector<int> filenames;
  filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
  if (filenames.size () != 2)
  {
    std::cout << "Usage: input_file_path.pcd output_file_path.pcd\n";
    exit (-1);
  }
  input_filename_ = argv[filenames.at(0)];
  output_filename_ = argv[filenames.at(1)];

  input_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  output_cloud_ = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_filename_, *input_cloud_) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }
  std::cout << "Loaded "
            << input_cloud_->width * input_cloud_->height
            << " data points from input pcd" << std::endl;
}

void BasicIOPCD::execute()
{
  // Overwrite this method and access the class member 'input_cloud_'
  // to access the input cloud that has been passed as a parameter
  // Save the finished pointcloud to 'output_cloud_'
  // and call write_pcd() to write it
  output_cloud_ = input_cloud_;
}

void BasicIOPCD::write_pcd()
{
  // write pcd
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << output_filename_;
  writer.write(ss.str(), *output_cloud_);
  std::cerr << "Saved " << output_cloud_->points.size () << " data points" << std::endl;
}
