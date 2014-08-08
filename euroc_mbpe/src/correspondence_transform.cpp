#include "euroc_mbpe/correspondence_transform.h"

Eigen::VectorXf CorrespondenceTransform::getVectorFromPointCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr p)
{
  if(p->points.size() != 4)
  {
    std::cout << "Encountered a pointcloud that's not of size 4 in getVectorFromPointCloud4" << std::endl;
    return Eigen::Matrix< float, 12, 1 >::Zero();
  }

  Eigen::VectorXf result = Eigen::Matrix< float, 12, 1 >::Zero();
  for(int i=0; i<4; i++)
  {
    result[0+(i*3)] = p->points[i].x;
    result[1+(i*3)] = p->points[i].y;
    result[2+(i*3)] = p->points[i].z;
  }
  return result;
}

Eigen::Matrix4f CorrespondenceTransform::getRotationMatrixFromPose(Eigen::Matrix< float, 6, 1 > pose)
{
  // The first three elments of the given vector are the
  // rotations around x,y and z
  
  Eigen::Matrix3f rz = Eigen::Matrix3f::Identity();
  float theta = pose[2];
  rz (0,0) = cos (theta);
  rz (0,1) = -sin(theta);
  rz (1,0) = sin (theta);
  rz (1,1) = cos (theta);

  Eigen::Matrix3f ry = Eigen::Matrix3f::Identity();
  theta = pose[1];
  ry (0,0) = cos (theta);
  ry (2,0) = -sin(theta);
  ry (0,2) = sin (theta);
  ry (2,2) = cos (theta);

  Eigen::Matrix3f rx = Eigen::Matrix3f::Identity();
  theta = pose[0];
  rx (1,1) = cos (theta);
  rx (1,2) = -sin(theta);
  rx (2,1) = sin (theta);
  rx (2,2) = cos (theta);

  Eigen::Matrix3f rotation = rz * ry * rx;
  Eigen::Matrix4f result;
  // Copy rotational result
  result(0,0) = rotation(0,0);
  result(0,1) = rotation(0,1);
  result(0,2) = rotation(0,2);
  result(1,0) = rotation(1,0);
  result(1,1) = rotation(1,1);
  result(1,2) = rotation(1,2);
  result(2,0) = rotation(2,0);
  result(2,1) = rotation(2,1);
  result(2,2) = rotation(2,2);
  // Apply translation
  // These are the last three elements in the vector.
  result(0,3) = pose[3];
  result(1,3) = pose[4];
  result(2,3) = pose[5];
  result(3,3) = 0; // Set last translation element to 0

  return result;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CorrespondenceTransform::transformVariedPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    Eigen::Matrix< float, 6, 1 > pose, int variation_idx, float e)
{
  // Transform the points with a slight change of param1 ...
  pcl::PointCloud<pcl::PointXYZ>::Ptr pts_param1 (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix< float, 6, 1 > varied_vector = Eigen::Matrix< float, 6, 1 >::Zero();
  varied_vector[variation_idx] = e;
  Eigen::Matrix< float, 6, 1 > pose_varied = pose + varied_vector;    
  pcl::transformPointCloud(*cloud, *pts_param1, getRotationMatrixFromPose(pose_varied) );
  return pts_param1;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CorrespondenceTransform::generateBoxModelFromPose(Eigen::Matrix< float, 6, 1 > pose){
  pcl::PointCloud<pcl::PointXYZ>::Ptr result (new pcl::PointCloud<pcl::PointXYZ>);
  result->width=8;
  result->height=1;
  result->is_dense=true;
  // result->points.resize(8);
  double box_size=0.6;
  double box_size_half=0.3;

  for(double x=-box_size_half; x<= box_size_half; x+=box_size)
  {
    for(double y=-box_size_half; y<= box_size_half; y+=box_size)
    {
      for(double z=-box_size_half; z<= box_size_half; z+=box_size)
      {
        pcl::PointXYZ c;
        c.x = x;
        c.y = y;
        c.z = z; 
        std::cout << c << std::endl;
        result->push_back(c);
      }
    }
  }
}
