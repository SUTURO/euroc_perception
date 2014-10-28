#include <suturo_perception_cad_recognition/IA_method.h>

using namespace suturo_perception;

Eigen::Matrix< float, 4, 4 > IAMethod::rotateAroundCrossProductOfNormals(
    Eigen::Vector3f base_normal,
    Eigen::Vector3f normal_to_rotate,
    bool store_transformation)
{
  normal_to_rotate *= -1; // The model is standing upside down, rotate the normal by 180 DEG
  float costheta = normal_to_rotate.dot(base_normal) / (normal_to_rotate.norm() * base_normal.norm() );

  Eigen::Vector3f axis;
  Eigen::Vector3f firstAxis = normal_to_rotate.cross(base_normal);
  firstAxis.normalize();
  axis=firstAxis;
  float c = costheta;
  std::cout << "rotate COSTHETA: " << acos(c) << " RAD, " << ((acos(c) * 180) / M_PI) << " DEG" << std::endl;
  float s = sqrt(1-c*c);
  float CO = 1-c;

  float x = axis(0);
  float y = axis(1);
  float z = axis(2);

  Eigen::Matrix< float, 4, 4 > rotationBox;
  rotationBox(0,0) = x*x*CO+c;
  rotationBox(1,0) = y*x*CO+z*s;
  rotationBox(2,0) = z*x*CO-y*s;

  rotationBox(0,1) = x*y*CO-z*s;
  rotationBox(1,1) = y*y*CO+c;
  rotationBox(2,1) = z*y*CO+x*s;

  rotationBox(0,2) = x*z*CO+y*s;
  rotationBox(1,2) = y*z*CO-x*s;
  rotationBox(2,2) = z*z*CO+c;
  // Translation vector
  rotationBox(0,3) = 0;
  rotationBox(1,3) = 0;
  rotationBox(2,3) = 0;

  // The rest of the 4x4 matrix
  rotationBox(3,0) = 0;
  rotationBox(3,1) = 0;
  rotationBox(3,2) = 0;
  rotationBox(3,3) = 1;

  if(store_transformation)
    rotations_.push_back(rotationBox);

  return rotationBox;
}

Eigen::Matrix< float, 4, 4> IAMethod::getTranslationMatrix(
    float x, float y, float z)
{
  Eigen::Matrix< float, 4, 4> translation;

  translation(0,0) = 1;
  translation(1,0) = 0;
  translation(2,0) = 0;

  translation(0,1) = 0;
  translation(1,1) = 1;
  translation(2,1) = 0;

  translation(0,2) = 0;
  translation(1,2) = 0;
  translation(2,2) = 1;
  // Translation vector
  translation(0,3) = x;
  translation(1,3) = y;
  translation(2,3) = z;

  // The rest of the 4x4 matrix
  translation(3,0) = 0;
  translation(3,1) = 0;
  translation(3,2) = 0;
  translation(3,3) = 1;

  return translation;
}


Cuboid IAMethod::computeCuboidFromBorderPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points)
{
  Cuboid c;

  // Get the "width": (minx,miny) -> (maxx,miny)
  c.length1 = pcl::distances::l2(corner_points->points.at(0).getVector4fMap(),corner_points->points.at(2).getVector4fMap());
  // Get the "height": (minx,miny) -> (minx,maxy)
  c.length2 = pcl::distances::l2(corner_points->points.at(0).getVector4fMap(),corner_points->points.at(3).getVector4fMap());
  // Get the "depth": (minx,miny,minz) -> (minx,maxy,maxz)
  c.length3 = pcl::distances::l2(corner_points->points.at(0).getVector4fMap(),corner_points->points.at(4).getVector4fMap());

  c.volume = c.length1 * c.length2 * c.length3;
  c.corner_points = corner_points;
  return c;
}

void IAMethod::computeCuboidCornersWithMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points)
{
  pcl::PointXYZ min_pt, max_pt;
  pcl::getMinMax3D(*cloud_in, min_pt, max_pt);
  // Compute the bounding box edge points
  pcl::PointXYZRGB pt1;
  pt1.x = min_pt.x; pt1.y = min_pt.y; pt1.z = min_pt.z;

  pcl::PointXYZRGB pt2;
  pt2.x = max_pt.x; pt2.y = max_pt.y; pt2.z = max_pt.z;
  pcl::PointXYZRGB pt3;
  pt3.x = max_pt.x,pt3.y = min_pt.y,pt3.z = min_pt.z;
  pcl::PointXYZRGB pt4;
  pt4.x = min_pt.x,pt4.y = max_pt.y,pt4.z = min_pt.z;
  pcl::PointXYZRGB pt5;
  pt5.x = min_pt.x,pt5.y = min_pt.y,pt5.z = max_pt.z;

  pcl::PointXYZRGB pt6;
  pt6.x = min_pt.x,pt6.y = max_pt.y,pt6.z = max_pt.z;
  pcl::PointXYZRGB pt7;
  pt7.x = max_pt.x,pt7.y = max_pt.y,pt7.z = min_pt.z;
  pcl::PointXYZRGB pt8;
  pt8.x = max_pt.x,pt8.y = min_pt.y,pt8.z = max_pt.z;

  corner_points->push_back(pt1);
  corner_points->push_back(pt2);
  corner_points->push_back(pt3);
  corner_points->push_back(pt4);
  corner_points->push_back(pt5);
  corner_points->push_back(pt6);
  corner_points->push_back(pt7);
  corner_points->push_back(pt8);
}
