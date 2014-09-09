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
#include <suturo_msgs/Task.h>

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

  std::vector<suturo_msgs::Shape> shapes;
  suturo_msgs::Shape shape1;
  shape1.type = shape1.BOX;
  // 0.05 x 0.05 x 0.05
  shape1.dimensions.push_back(0.05f);
  shape1.dimensions.push_back(0.05f);
  shape1.dimensions.push_back(0.05f);
  shape1.pose.linear.x = 0;
  shape1.pose.linear.y = 0;
  shape1.pose.linear.z = 0;
  shape1.pose.angular.x = 0;
  shape1.pose.angular.y = 0;
  shape1.pose.angular.z = 0;

  // Define the extra parts on the handlebar
  suturo_msgs::Shape shape2;
  shape2.type = shape1.BOX;
  // 0.05 x 0.05 x 0.05
  shape2.dimensions.push_back(0.05f);
  shape2.dimensions.push_back(0.05f);
  shape2.dimensions.push_back(0.05f);
  shape2.pose.linear.x = 0;
  shape2.pose.linear.y = 0;
  shape2.pose.linear.z = 0.35f;
  shape2.pose.angular.x = 0;
  shape2.pose.angular.y = 0;
  shape2.pose.angular.z = 0;

  suturo_msgs::Shape shape3;
  shape3.type = shape1.CYLINDER;
  shape3.dimensions.push_back(0.3f);
  shape3.dimensions.push_back(0.01f);
  shape3.pose.linear.x = 0;
  shape3.pose.linear.y = 0;
  shape3.pose.linear.z = 0.175f;
  shape3.pose.angular.x = 0;
  shape3.pose.angular.y = 0;
  shape3.pose.angular.z = 0;

  shapes.push_back(shape1);
  shapes.push_back(shape2);
  shapes.push_back(shape3);

  // output_cloud = generateCylinder(0.1, 0.02, 5000);
  // output_cloud = g.generateComposed();
  output_cloud = g.generateComposed(shapes);
  // g.generateComposed(shapes);

  // output_cloud = g.generateBox(0.05, 0.05, 0.05, 4000);

  // write pcd
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << output_filename;
  writer.write(ss.str(), *output_cloud);
  std::cerr << "Saved " << output_cloud->points.size () << " data points" << std::endl;

  return (0);
}

