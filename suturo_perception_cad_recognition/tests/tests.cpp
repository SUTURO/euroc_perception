#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <boost/program_options.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
// #include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
// #include <pcl/registration/ia_ransac.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
// #include <suturo_perception_utils.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/features/fpfh.h>
// #include <pcl/features/shot.h>
#include <suturo_perception_match_cuboid/cuboid_matcher.h>
// #include <suturo_perception_cad_recognition/icp_fitter.h>
#include <boost/algorithm/string.hpp>
#include <suturo_perception_cad_recognition/model_pose_estimation.h>
#include <shape_msgs/SolidPrimitive.h>
#include <perception_utils/capability.hpp>

#define CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR 0.005f
#define CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR 0.04f

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

TEST(suturo_perception_mbpe, pose_estimation_cube)
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
  obj.primitives.push_back(shape1);
  obj.primitive_poses.push_back(pose1);
  objects->push_back(obj);

  suturo_perception::PipelineData::Ptr data_;
  suturo_perception::PipelineObject::Ptr object_;
  ModelPoseEstimation mpe(objects,data_,object_);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal);
  mpe.setVoxelSize(0.003f);
  mpe.execute();

  std::cout << "Fitness for cube matching: " << mpe.getFitnessScore() << std::endl;
  // The estimation should be succesful
	ASSERT_TRUE( mpe.poseEstimationSuccessful() );
  // Check the pose (origin.x, origin.y, origin.z)
  ASSERT_NEAR( mpe.getEstimatedPose()[0], -0.0634356, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[1], -0.0439471, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[2],    0.73877, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );

  // Check the orientation (x,y,z,w from a Quaternionf)
  ASSERT_NEAR( mpe.getEstimatedPose()[3],  0.813566, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[4], -0.164979, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[5],  0.309649, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[6], -0.463691, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );

  SUCCEED();
}

TEST(suturo_perception_mbpe, pose_estimation_cylinder)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_mbpe");
  // std::string modelpath  = "test_files/005box_4000pts.pcd";
  std::string objectpath = "test_files/correctly_segmented_cylinder.pcd";
  std::string object_table_normal_path = "test_files/correctly_segmented_cylinder.pcd_table_normal";

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
  obj.name="green_cylinder";
  obj.color="00ff00";
  obj.description="a green cylinder";
  obj.surface_material = suturo_msgs::Object::ALUMINIUM;

  shape_msgs::SolidPrimitive shape1;
	geometry_msgs::Pose pose1;
  shape1.type = shape1.CYLINDER;
  // 0.1 x 0.02
  shape1.dimensions.push_back(0.1f);
  shape1.dimensions.push_back(0.02f);
  pose1.position.x = 0;
  pose1.position.y = 0;
  pose1.position.z = 0;
  pose1.orientation.x = 0;
  pose1.orientation.y = 0;
  pose1.orientation.z = 0;
  pose1.orientation.w = 1;
  obj.primitives.push_back(shape1);
  obj.primitive_poses.push_back(pose1);
  objects->push_back(obj);

  suturo_perception::PipelineData::Ptr data_;
  suturo_perception::PipelineObject::Ptr object_;
  ModelPoseEstimation mpe(objects,data_,object_);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal);
  // mpe.setDumpICPFitterPointclouds(true);
  mpe.setVoxelSize(0.003f);
  mpe.execute();

  std::cout << "Fitness for cylinder matching: " << mpe.getFitnessScore() << std::endl;
  // The estimation should be succesful
	ASSERT_TRUE( mpe.poseEstimationSuccessful() );
  // Check the pose (origin.x, origin.y, origin.z)
  ASSERT_NEAR( mpe.getEstimatedPose()[0],  -0.110493, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[1],   0.163387, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[2],   0.566805, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );

  // Check the orientation (x,y,z,w from a Quaternionf)
  ASSERT_NEAR( mpe.getEstimatedPose()[3],  -0.270409, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[4], 0.00236567, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[5],  -0.227, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[6],   0.910853, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );

  SUCCEED();
}

TEST(suturo_perception_mbpe, pose_estimation_handlebar)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_mbpe");
  // std::string modelpath  = "test_files/005box_4000pts.pcd";
  std::string objectpath = "test_files/correctly_segmented_handlebar.pcd";
  std::string object_table_normal_path = "test_files/correctly_segmented_handlebar.pcd_table_normal";

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
  obj.name="blue_handle";
  obj.color="0000ff";
  obj.description="a blue compound of a cylinder with two cubes";
  obj.surface_material = suturo_msgs::Object::ALUMINIUM;

  shape_msgs::SolidPrimitive shape1, shape2, shape3;
	geometry_msgs::Pose pose1, pose2, pose3;
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
  shape2.type = shape1.BOX;
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

  shape3.type = shape1.CYLINDER;
  shape3.dimensions.push_back(0.3f);
  shape3.dimensions.push_back(0.01f);
  pose3.position.x = 0;
  pose3.position.y = 0;
  pose3.position.z = 0.175f;
  pose3.orientation.x = 0;
  pose3.orientation.y = 0;
  pose3.orientation.z = 0;
  pose3.orientation.w = 1;


  obj.primitives.push_back(shape1);
  obj.primitives.push_back(shape2);
  obj.primitives.push_back(shape3);
  obj.primitive_poses.push_back(pose1);
  obj.primitive_poses.push_back(pose2);
  obj.primitive_poses.push_back(pose3);
  objects->push_back(obj);

  suturo_perception::PipelineData::Ptr data_;
  suturo_perception::PipelineObject::Ptr object_;
  ModelPoseEstimation mpe(objects,data_,object_);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal);
  mpe.setVoxelSize(0.003f);
  mpe.setDumpICPFitterPointclouds(true); // Enable debugging. This will save pointclouds
  mpe.execute();

  std::cout << "Fitness for handlebar matching: " << mpe.getFitnessScore() << std::endl;
  // The estimation should be succesful
	ASSERT_TRUE( mpe.poseEstimationSuccessful() );
  // Check the pose (origin.x, origin.y, origin.z)
  ASSERT_NEAR( mpe.getEstimatedPose()[0],  -0.320974, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[1],  -0.100329, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[2],    1.02429, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );

  // Check the orientation (x,y,z,w from a Quaternionf)
  ASSERT_NEAR( mpe.getEstimatedPose()[3],   0.876425, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[4], -0.0972819, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[5],    0.17143, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[6],  -0.439349, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );

  SUCCEED();
}

TEST(suturo_perception_mbpe, nan_handling)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // Fill in the cloud data
  input_cloud->width    = 5;
  input_cloud->height   = 1;
  input_cloud->is_dense = false;
  input_cloud->points.resize (input_cloud->width * input_cloud->height);

  for (size_t i = 0; i < input_cloud->points.size ()-1; ++i)
  {
    input_cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    input_cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    input_cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  // Produce NaN
  input_cloud->points[4].x = 0.0 / 0.0;
  input_cloud->points[4].y = 0.0 / 0.0;
  input_cloud->points[4].z = 0.0 / 0.0;

  // std::string package_path = ros::package::getPath("suturo_perception_mbpe");
  // // std::string modelpath  = "test_files/005box_4000pts.pcd";
  // std::string objectpath = "test_files/correctly_segmented_box.pcd";
  // std::string object_table_normal_path = "test_files/correctly_segmented_box.pcd_table_normal";

  // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + objectpath, *input_cloud) == -1)
  // {
  //   PCL_ERROR ("Couldn't read input file\n");
  //   exit (-1);
  // }

  std::string package_path = ros::package::getPath("suturo_perception_mbpe");
  std::string object_table_normal_path = "test_files/correctly_segmented_box.pcd_table_normal";
  Eigen::Vector4f table_normal = getTableNormalFromStringLine(object_table_normal_path,package_path);
  std::cout << "Using table normal: " << table_normal << std::endl;

  // Prepare the model that should be matched against the input cloud
  boost::shared_ptr<std::vector<suturo_msgs::Object> > objects(new std::vector<suturo_msgs::Object>);
  suturo_msgs::Object obj;
  obj.name="red_cube";
  obj.color="ff0000";
  obj.description="a red cube";
  obj.surface_material = suturo_msgs::Object::ALUMINIUM;

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
  obj.primitives.push_back(shape1);
  obj.primitive_poses.push_back(pose1);
  objects->push_back(obj);

  suturo_perception::PipelineData::Ptr data_;
  suturo_perception::PipelineObject::Ptr object_;
  ModelPoseEstimation mpe(objects,data_,object_);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal);
  mpe.setRemoveNaNs(true);
  mpe.execute();

  std::cout << "Fitness for nan matching: " << mpe.getFitnessScore() << std::endl;
  // The estimation should be succesful
	ASSERT_FALSE( mpe.poseEstimationSuccessful() );
  SUCCEED();
}

TEST(suturo_perception_mbpe, pose_estimation_box_against_multiple_models)
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
  obj.primitives.push_back(shape1);
  obj.primitive_poses.push_back(pose1);


  suturo_msgs::Object cobj;
  cobj.name="green_cylinder";
  cobj.color="00ff00";
  cobj.description="a green cylinder";
  cobj.surface_material = suturo_msgs::Object::ALUMINIUM;

  shape_msgs::SolidPrimitive shape2;
  geometry_msgs::Pose pose2;
  shape2.type = shape2.CYLINDER;
  // 0.1 x 0.02
  shape2.dimensions.push_back(0.1f);
  shape2.dimensions.push_back(0.02f);
  pose2.position.x = 0;
  pose2.position.y = 0;
  pose2.position.z = 0;
  pose2.orientation.x = 0;
  pose2.orientation.y = 0;
  pose2.orientation.z = 0;
  pose2.orientation.w = 1;
  cobj.primitives.push_back(shape2);
  cobj.primitive_poses.push_back(pose2);

  objects->push_back(obj);
  objects->push_back(cobj);

  suturo_perception::PipelineData::Ptr data_;
  suturo_perception::PipelineObject::Ptr object_;
  ModelPoseEstimation mpe(objects,data_,object_);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal);
  mpe.setVoxelSize(0.003f);
  mpe.execute();

  std::cout << "Fitness for cube and cylinder matching: " << mpe.getFitnessScore() << std::endl;
  // The estimation should be succesful
	ASSERT_TRUE( mpe.poseEstimationSuccessful() );
  // Check the pose (origin.x, origin.y, origin.z)
  ASSERT_NEAR( mpe.getEstimatedPose()[0], -0.0634356, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[1], -0.0439471, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[2],    0.73877, CAD_RECOGNITION_TEST_ACCEPTABLE_POINT_ERROR );

  // Check the orientation (x,y,z,w from a Quaternionf)
  ASSERT_NEAR( mpe.getEstimatedPose()[3],  0.813566, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[4], -0.164979, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[5],  0.309649, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );
  ASSERT_NEAR( mpe.getEstimatedPose()[6], -0.463691, CAD_RECOGNITION_TEST_ACCEPTABLE_ORIENTATION_ERROR );

  SUCCEED();
}

TEST(suturo_perception_mbpe, pose_estimation_box_against_multiple_models_but_ignore_the_best)
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
  obj.primitives.push_back(shape1);
  obj.primitive_poses.push_back(pose1);


  suturo_msgs::Object cobj;
  cobj.name="green_cylinder";
  cobj.color="00ff00";
  cobj.description="a green cylinder";
  cobj.surface_material = suturo_msgs::Object::ALUMINIUM;

  shape_msgs::SolidPrimitive shape2;
  geometry_msgs::Pose pose2;
  shape2.type = shape2.CYLINDER;
  // 0.1 x 0.02
  shape2.dimensions.push_back(0.1f);
  shape2.dimensions.push_back(0.02f);
  pose2.position.x = 0;
  pose2.position.y = 0;
  pose2.position.z = 0;
  pose2.orientation.x = 0;
  pose2.orientation.y = 0;
  pose2.orientation.z = 0;
  pose2.orientation.w = 1;
  cobj.primitives.push_back(shape2);
  cobj.primitive_poses.push_back(pose2);

  objects->push_back(obj);
  objects->push_back(cobj);

  suturo_perception::PipelineData::Ptr data_;
  suturo_perception::PipelineObject::Ptr object_;
  ModelPoseEstimation mpe(objects,data_,object_);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal);
  mpe.setVoxelSize(0.003f);
  std::vector<int> modelsOfInterest;
  modelsOfInterest.push_back(1); // only match against the cylinder (and fail)
  mpe.setModelsOfInterest(modelsOfInterest);
  mpe.execute();

  // The estimation should be succesful
	ASSERT_FALSE( mpe.poseEstimationSuccessful() );

  SUCCEED();
}

TEST(suturo_perception_mbpe, empty_pointcloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  std::string package_path = ros::package::getPath("suturo_perception_mbpe");
  std::string object_table_normal_path = "test_files/correctly_segmented_box.pcd_table_normal";
  Eigen::Vector4f table_normal = getTableNormalFromStringLine(object_table_normal_path,package_path);
  std::cout << "Using table normal: " << table_normal << std::endl;

  // Prepare the model that should be matched against the input cloud
  boost::shared_ptr<std::vector<suturo_msgs::Object> > objects(new std::vector<suturo_msgs::Object>);
  suturo_msgs::Object obj;
  obj.name="red_cube";
  obj.color="ff0000";
  obj.description="a red cube";
  obj.surface_material = suturo_msgs::Object::ALUMINIUM;

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
  obj.primitives.push_back(shape1);
  obj.primitive_poses.push_back(pose1);
  objects->push_back(obj);

  suturo_perception::PipelineData::Ptr data_;
  suturo_perception::PipelineObject::Ptr object_;
  ModelPoseEstimation mpe(objects,data_,object_);
  mpe.setInputCloud(input_cloud);
  mpe.setSurfaceNormal(table_normal); // Test this
  // mpe.setRemoveNaNs(true);
  mpe.execute();

  // std::cout << "Fitness for nan matching: " << mpe.getFitnessScore() << std::endl;
  // The estimation should be succesful
	ASSERT_FALSE( mpe.poseEstimationSuccessful() );
  SUCCEED();
}

// TODO
// Tests for
// - no surface normal set
// - no input pointcloud set

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 

