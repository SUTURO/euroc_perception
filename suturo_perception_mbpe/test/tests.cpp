#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


TEST(suturo_perception_match_cuboid, match_corny_with_table)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  std::string package_path = ros::package::getPath("suturo_perception_mbpe");
  std::string filepath = "test_data/corny_plane_mode.pcd";
  // pcl::ModelCoefficients::Ptr table_coefficients (new pcl::ModelCoefficients);

  // if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (package_path + "/" + filepath, *input_cloud) == -1)
  // {
  //   PCL_ERROR ("Couldn't read input file\n");
  //   exit (-1);
  // }
	ASSERT_TRUE( true );
  

  SUCCEED();
}

int main(int argc, char **argv) 
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 

