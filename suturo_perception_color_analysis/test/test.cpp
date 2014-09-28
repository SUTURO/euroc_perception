#include "suturo_perception_color_analysis/color_analysis.h"
#include <perception_utils/pipeline_object.hpp>
#include <iostream>
#include <sstream>
#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace suturo_perception;

TEST(color_analysis_test, color_1_test)
{
  PipelineObject::Ptr p = PipelineObject::Ptr(new PipelineObject());
  PipelineData::Ptr d = PipelineData::Ptr(new PipelineData());
  ColorAnalysis ca(d, p);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  
  pcl::PointXYZRGB point1, point2, point3;
  uint8_t r = 255, g = 0, b = 0;
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  point1.rgb = *reinterpret_cast<float*>(&rgb);
  point2.rgb = *reinterpret_cast<float*>(&rgb);
  point3.rgb = *reinterpret_cast<float*>(&rgb);
  cloud->points.push_back(point1);
  cloud->points.push_back(point2);
  cloud->points.push_back(point3);
  
  HSVColor averageColorHSV = ca.getAverageColorHSV(cloud);
  ASSERT_EQ(0, averageColorHSV.h);
  ASSERT_EQ(1.0, averageColorHSV.s);
  ASSERT_EQ(1.0, averageColorHSV.v);
}

TEST(color_analysis_test, color_2_test)
{
  PipelineObject::Ptr p = PipelineObject::Ptr(new PipelineObject());
  PipelineData::Ptr d = PipelineData::Ptr(new PipelineData());
  ColorAnalysis ca(d, p);

  uint8_t r = 255, g = 100, b = 150;
  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  HSVColor hsv = ca.convertRGBToHSV(rgb);
  ASSERT_EQ(341, hsv.h);
  ASSERT_LT(0.60, hsv.s);
  ASSERT_GT(0.61, hsv.s);
  ASSERT_EQ(1.0, hsv.v);


  //ASSERT_EQ(0x000000, ca.convertRGBToHSV(0x000000)); // black
  //ASSERT_EQ(0x0000ff, ca.convertRGBToHSV(0xffffff)); // white
  //ASSERT_EQ(0x00ffff, ca.convertRGBToHSV(0xff0000)); // red
  //ASSERT_EQ(0x55ffff, ca.convertRGBToHSV(0x00ff00)); // green

  ASSERT_EQ(0xff0000, ca.convertHSVToRGB(ca.convertRGBToHSV(0xff0000))); // black
  ASSERT_EQ(0xffffff, ca.convertHSVToRGB(ca.convertRGBToHSV(0xffffff))); // white
  ASSERT_EQ(0xff0000, ca.convertHSVToRGB(ca.convertRGBToHSV(0xff0000))); // red
  ASSERT_EQ(0x00ff00, ca.convertHSVToRGB(ca.convertRGBToHSV(0x00ff00))); // green
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
