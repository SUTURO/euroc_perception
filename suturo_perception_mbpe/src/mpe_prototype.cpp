#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>
#include <suturo_perception_mbpe/model_pose_estimation.h>

Eigen::Vector4f getTableNormalFromStringLine(std::string object_table_normal, std::string package_path)
{

  // Process table normal
  std::ifstream file( (package_path + "/" + object_table_normal).c_str() );
  std::string table_normal_string; 
  if(!std::getline(file, table_normal_string))
  {
    std::cout << "The table normal file can't be read";
    exit (-1);
  } 
  std::vector<std::string> table_normal_components;
  boost::split(table_normal_components, table_normal_string, boost::is_any_of(","));

  if(table_normal_components.size() != 4)
  {
    std::cout << "Extracted " << table_normal_components.size() << " components from the table normal file: " << object_table_normal << ". Required are exactly 4! See the pcl ModelCoefficients or a RANSAC plane for reference" << std::endl;
    exit (-1);
  }
  Eigen::Vector4f table_normal(
      atof(table_normal_components.at(0).c_str()),
      atof(table_normal_components.at(1).c_str()),
      atof(table_normal_components.at(2).c_str()),
      atof(table_normal_components.at(3).c_str())
      );
  return table_normal;
}


int main(int argc, const char *argv[])
{
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_mbpe");
  // std::string modelpath  = "test_files/005box_4000pts.pcd";
  std::string objectpath = "test_files/correctly_segmented_box.pcd";
  std::string object_table_normal_path = "test_files/correctly_segmented_box.pcd_table_normal";

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + objectpath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  Eigen::Vector4f table_normal = getTableNormalFromStringLine(object_table_normal_path,package_path);
  std::cout << "Using table normal: " << table_normal << std::endl;

  // Prepare the model that should be matched against the input cloud
  boost::shared_ptr<std::vector<suturo_msgs::Object> > objects(new std::vector<suturo_msgs::Object>);
  suturo_msgs::Object obj;
  obj.name="red_cube";
  obj.color="ff0000";
  obj.description="a red cube";
  obj.surface_material = suturo_msgs::Object::ALUMINIUM;

  suturo_msgs::Shape shape1;
  shape1.shape_type = shape1.BOX;
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
  obj.shapes.push_back(shape1);
  objects->push_back(obj);
  // shapes->push_back(shape1);

  ModelPoseEstimation mpe(objects);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal);
  mpe.setVoxelSize(0.003f);
  mpe.execute();

  std::cout << mpe.getFitnessScore() << std::endl;
  return 0;
}
