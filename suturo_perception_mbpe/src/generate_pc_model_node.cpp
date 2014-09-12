#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/filter.h>
#include <pcl/common/transforms.h>
#include <math.h>
#include <suturo_perception_mbpe/generate_pc_model.h>
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

  std::vector<shape_msgs::SolidPrimitive> primitives;
  std::vector<geometry_msgs::Pose> primitive_poses;
	shape_msgs::SolidPrimitive shape1;
	geometry_msgs::Pose pose1;
  shape1.type = shape1.BOX;
  // 0.05 x 0.05 x 0.05
  shape1.dimensions.push_back(0.05f);
  shape1.dimensions.push_back(0.05f);
  shape1.dimensions.push_back(0.05f);
  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.position.z = 0;
  pose1.orientation.x = 0;
  pose1.orientation.y = 0;
  pose1.orientation.z = 0;
  pose1.orientation.w = 1;

  // Define the extra parts on the handlebar
  shape_msgs::SolidPrimitive shape2;
	geometry_msgs::Pose pose2;
  shape2.type = shape2.BOX;
  // 0.05 x 0.05 x 0.05
  shape2.dimensions.push_back(0.05f);
  shape2.dimensions.push_back(0.05f);
  shape2.dimensions.push_back(0.05f);
  pose2.position.x = 0;
  pose2.position.y = 0;
  pose2.position.z = 0.35f;
  pose2.orientation.x = 0;
  pose2.orientation.y = 0;
  pose2.orientation.z = 0;
  pose2.orientation.w = 1;

  shape_msgs::SolidPrimitive shape3;
	geometry_msgs::Pose pose3;
  shape3.type = shape3.CYLINDER;
  shape3.dimensions.push_back(0.3f);
  shape3.dimensions.push_back(0.01f);
  pose3.position.x = 0;
  pose3.position.y = 0;
  pose3.position.z = 0.175f;
  pose3.orientation.x = 0;
  pose3.orientation.y = 0;
  pose3.orientation.z = 0;
  pose3.orientation.w = 1;

  primitives.push_back(shape1);
  primitives.push_back(shape2);
  primitives.push_back(shape3);

  primitive_poses.push_back(pose1);
  primitive_poses.push_back(pose2);
  primitive_poses.push_back(pose3);

  // output_cloud = generateCylinder(0.1, 0.02, 5000);
  // output_cloud = g.generateComposed();
  output_cloud = g.generateComposed(primitives, primitive_poses);
  // g.generateComposed(primitives);

  // output_cloud = g.generateBox(0.05, 0.05, 0.05, 4000);

  // write pcd
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << output_filename;
  writer.write(ss.str(), *output_cloud);
  std::cerr << "Saved " << output_cloud->points.size () << " data points" << std::endl;

  return (0);
}

