#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

TEST(suturo_perception_match_cuboid, match_cuboid_on_empty_cloud)
{

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

  // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (input_filename, *input_cloud) == -1) //* load the file
  // {
  //   PCL_ERROR ("Couldn't read input file\n");
  //   exit (-1);
  // }
  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  // cm.setSaveIntermediateResults(true);
  Cuboid cuboid;
  cm.execute(cuboid);


	ASSERT_FALSE( cm.estimationSuccessful() );
	ASSERT_TRUE( cm.getDetectedPlanes()->size() == 0);
	ASSERT_TRUE( cm.transformationCount() == 0);

  SUCCEED();
}

TEST(suturo_perception_match_cuboid, match_muesli_not_rotated)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/muesli_not_rotated.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  // cm.setSaveIntermediateResults(true);
  Cuboid cuboid;
  cm.execute(cuboid);


	ASSERT_TRUE( cm.estimationSuccessful() );
  // Width: 0.150506 
	ASSERT_TRUE( cuboid.length1 > 0.14);
	ASSERT_TRUE( cuboid.length1 < 0.16);

  // Height: 0.208633 
	ASSERT_TRUE( cuboid.length2 > 0.19);
	ASSERT_TRUE( cuboid.length2 < 0.22);

  // Depth: 0.0701288 
	ASSERT_TRUE( cuboid.length3 > 0.06);
	ASSERT_TRUE( cuboid.length3 < 0.08);
  
  // Volume: 0.00220208 m^3
	ASSERT_TRUE( cuboid.volume > 0.002);
	ASSERT_TRUE( cuboid.volume < 0.003);

  SUCCEED();
}



TEST(suturo_perception_match_cuboid, match_muesli_rotated)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/muesli_rotated.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  // cm.setSaveIntermediateResults(true);
  Cuboid cuboid;
  cm.execute(cuboid);


	ASSERT_TRUE( cm.estimationSuccessful() );
  // Width: 0.210852 
	ASSERT_TRUE( cuboid.length1 > 0.20);
	ASSERT_TRUE( cuboid.length1 < 0.22);

  // Height: 0.142768 
	ASSERT_TRUE( cuboid.length2 > 0.13);
	ASSERT_TRUE( cuboid.length2 < 0.15);

  // Depth: 0.0841869 
	ASSERT_TRUE( cuboid.length3 > 0.07);
	ASSERT_TRUE( cuboid.length3 < 0.09);
  
  // Volume: 0.00253427 m^3 
	ASSERT_TRUE( cuboid.volume > 0.002);
	ASSERT_TRUE( cuboid.volume < 0.003);

  SUCCEED();
}


TEST(suturo_perception_match_cuboid, match_teabox)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/teabox.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  // cm.setSaveIntermediateResults(true);
  Cuboid cuboid;
  cm.execute(cuboid);

	ASSERT_TRUE( cm.estimationSuccessful() );
  // Width: 0.125149
	ASSERT_TRUE( cuboid.length1 > 0.12);
	ASSERT_TRUE( cuboid.length1 < 0.14);

  // Height: 0.0666196 
	ASSERT_TRUE( cuboid.length2 > 0.06);
	ASSERT_TRUE( cuboid.length2 < 0.08);

  // Depth: 0.0666039 
	ASSERT_TRUE( cuboid.length3 > 0.06);
	ASSERT_TRUE( cuboid.length3 < 0.08);
  
  // Volume: 0.0005553 m^3
	ASSERT_TRUE( cuboid.volume > 0.0005);
	ASSERT_TRUE( cuboid.volume < 0.0007);

  SUCCEED();
}



TEST(suturo_perception_match_cuboid, match_teabox_towards)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/teabox_towards.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  // cm.setSaveIntermediateResults(true);
  Cuboid cuboid;
  cm.execute(cuboid);

	ASSERT_TRUE( cm.estimationSuccessful() );

  // Width: 0.070335 
	ASSERT_TRUE( cuboid.length1 > 0.06);
	ASSERT_TRUE( cuboid.length1 < 0.08);

  // Height: 0.121144 
	ASSERT_TRUE( cuboid.length2 > 0.11);
	ASSERT_TRUE( cuboid.length2 < 0.13);

  // Depth: 0.065673 
	ASSERT_TRUE( cuboid.length3 > 0.06);
	ASSERT_TRUE( cuboid.length3 < 0.08);
  
  // Volume: 0.000559579 m^3
	ASSERT_TRUE( cuboid.volume > 0.0005);
	ASSERT_TRUE( cuboid.volume < 0.0007);

  SUCCEED();
}


TEST(suturo_perception_match_cuboid, unmatchable_basketball)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/basketball.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  // cm.setSaveIntermediateResults(true);
  Cuboid cuboid;
  cm.execute(cuboid);

	ASSERT_FALSE( cm.estimationSuccessful() );

  SUCCEED();
}

TEST(suturo_perception_match_cuboid, unmatchable_cleaner)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/cleaner.pcd";
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  // cm.setSaveIntermediateResults(true);
  Cuboid cuboid;
  cm.execute(cuboid);

	ASSERT_FALSE( cm.estimationSuccessful() );

  SUCCEED();
}

TEST(suturo_perception_match_cuboid, match_cafetfilter_with_table)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/cafetfilter_plane_mode.pcd";
  // pcl::ModelCoefficients::ptr table(-0.00567581,-0.782561,-0.622548,0.928668);
  pcl::ModelCoefficients::Ptr table_coefficients (new pcl::ModelCoefficients);

  // The rough normal of the table plane
  table_coefficients->values.push_back(-0.00567581);
  table_coefficients->values.push_back(-0.782561);
  table_coefficients->values.push_back(-0.622548);
  table_coefficients->values.push_back( 0.928668);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  cm.setTableCoefficients(table_coefficients);
  cm.setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
  Cuboid cuboid;
  cm.execute(cuboid);

	ASSERT_TRUE( cm.estimationSuccessful() );

  // Width: 0.1443
	ASSERT_TRUE( cuboid.length1 > 0.13);
	ASSERT_TRUE( cuboid.length1 < 0.15);

  // Height: 0.187366 
	ASSERT_TRUE( cuboid.length2 > 0.18);
	ASSERT_TRUE( cuboid.length2 < 0.20);

  // Depth: 0.051284  
	ASSERT_TRUE( cuboid.length3 > 0.04);
	ASSERT_TRUE( cuboid.length3 < 0.06);
  
  // Volume: 0.00138657 
	ASSERT_TRUE( cuboid.volume > 0.001);
	ASSERT_TRUE( cuboid.volume < 0.0015);

  SUCCEED();
}

TEST(suturo_perception_match_cuboid, match_baguette_with_table)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/baguette_plane_mode.pcd";
  // pcl::ModelCoefficients::ptr table(-0.00567581,-0.782561,-0.622548,0.928668);
  pcl::ModelCoefficients::Ptr table_coefficients (new pcl::ModelCoefficients);

  // The rough normal of the table plane
  table_coefficients->values.push_back(-0.00567581);
  table_coefficients->values.push_back(-0.782561);
  table_coefficients->values.push_back(-0.622548);
  table_coefficients->values.push_back( 0.928668);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  cm.setTableCoefficients(table_coefficients);
  cm.setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
  Cuboid cuboid;
  cm.execute(cuboid);

	ASSERT_TRUE( cm.estimationSuccessful() );

  
  // Width: 0.216117 
	ASSERT_TRUE( cuboid.length1 > 0.20);
	ASSERT_TRUE( cuboid.length1 < 0.22);

  // Height: 0.122619
  ASSERT_TRUE( cuboid.length2 > 0.12);
	ASSERT_TRUE( cuboid.length2 < 0.14);

  // Depth: 0.0442761 
	ASSERT_TRUE( cuboid.length3 > 0.04);
	ASSERT_TRUE( cuboid.length3 < 0.06);
  
  // Volume: 0.00117332 
	ASSERT_TRUE( cuboid.volume > 0.001);
	ASSERT_TRUE( cuboid.volume < 0.0015);

  SUCCEED();
}


TEST(suturo_perception_match_cuboid, match_corny_with_table)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_match_cuboid");
  std::string filepath = "test_data/corny_plane_mode.pcd";
  pcl::ModelCoefficients::Ptr table_coefficients (new pcl::ModelCoefficients);

  // The rough normal of the table plane
  table_coefficients->values.push_back(-0.00567581);
  table_coefficients->values.push_back(-0.782561);
  table_coefficients->values.push_back(-0.622548);
  table_coefficients->values.push_back( 0.928668);

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  {
    PCL_ERROR ("Couldn't read input file\n");
    exit (-1);
  }

  CuboidMatcher cm;
  cm.setInputCloud(input_cloud);
  cm.setDebug(false);
  cm.setTableCoefficients(table_coefficients);
  cm.setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
  Cuboid cuboid;
  cm.execute(cuboid);

	ASSERT_TRUE( cm.estimationSuccessful() );

 
  // Width: 0.144728 
	ASSERT_TRUE( cuboid.length1 > 0.14);
	ASSERT_TRUE( cuboid.length1 < 0.16);

  // Height: 0.134177 
  ASSERT_TRUE( cuboid.length2 > 0.13);
	ASSERT_TRUE( cuboid.length2 < 0.14);

  // Depth: 0.038689 
	ASSERT_TRUE( cuboid.length3 > 0.03);
	ASSERT_TRUE( cuboid.length3 < 0.05);
  
  // Volume: 0.000751307
	ASSERT_TRUE( cuboid.volume > 0.0007);
	ASSERT_TRUE( cuboid.volume < 0.0009);
  

  SUCCEED();
}


// TODO: Build test
// Estimated normal of the table: -0.0315049 -0.906697 -0.420605 0.772575
// cornflakes_on_table


int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
