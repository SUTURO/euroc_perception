#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <boost/program_options.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

typedef Eigen::Matrix< float, 6, 1 > 	Vector6f;
typedef Eigen::Matrix< float, 12, 1 > 	Vector12f;

Vector12f getVectorFromPointCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr p)
{
  if(p->points.size() != 4)
  {
    std::cout << "Encountered a pointcloud that's not of size 4 in getVectorFromPointCloud4" << std::endl;
    return Eigen::Matrix< float, 12, 1 >::Zero();
  }

  Vector12f result = Eigen::Matrix< float, 12, 1 >::Zero();
  for(int i=1; i<=4; i++)
  {
    result[0*i] = p->points[i-1].x;
    result[1*i] = p->points[i-1].y;
    result[2*i] = p->points[i-1].z;
  }
  return result;
}

Eigen::Matrix4f getRotationMatrixFromPose(Vector6f pose)
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

// Transform a cloud with a given pose + a small varation of one of the parameters in the pose
// variation_idx indicates which index should be added by e
pcl::PointCloud<pcl::PointXYZ>::Ptr transformVariedPose(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    Vector6f pose, int variation_idx, float e)
{
  // Transform the points with a slight change of param1 ...
  pcl::PointCloud<pcl::PointXYZ>::Ptr pts_param1 (new pcl::PointCloud<pcl::PointXYZ>);
  Vector6f varied_vector = Vector6f::Zero();
  // varied_vector[0] = varied_vector[1] = varied_vector[2] = varied_vector[3] = varied_vector[4] = varied_vector[5] = 0;
  varied_vector[0] = e;
  Vector6f pose_varied = pose + varied_vector;    
  pcl::transformPointCloud(*cloud, *pts_param1, getRotationMatrixFromPose(pose) );
  return pts_param1;
}

int main(int argc, const char *argv[])
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr model_correspondences (new pcl::PointCloud<pcl::PointXYZ>);
  // Init the model correspondences. Let's take some of the corners of a box
  pcl::PointXYZ c;
  c.x = -0.3;
  c.y = 0.3;
  c.z = -0.3; 
  model_correspondences->push_back(c);
  c.x = -0.3;
  c.y = 0.3;
  c.z = 0.3; 
  model_correspondences->push_back(c);
  c.x = -0.3;
  c.y = -0.3;
  c.z = 0.3; 
  model_correspondences->push_back(c);
  c.x = 0.3;
  c.y = -0.3;
  c.z = 0.3; 
  model_correspondences->push_back(c);

  // Transform the given points and think of these new points as the observed ones
  pcl::PointCloud<pcl::PointXYZ>::Ptr observed_correspondences (new pcl::PointCloud<pcl::PointXYZ>);
  Vector6f d; // (theta_x, theta_y, theta_z, x, y, z)
  d[0] = d[1] = d[2] = d[3] = d[4] = d[5] = 0;
  d[2] = M_PI/4;
  Eigen::Matrix4f transform_1 = getRotationMatrixFromPose(d);
  pcl::transformPointCloud(*model_correspondences, *observed_correspondences, transform_1 );

  // Initial guess for the pose
  Vector6f x;
  x[0] = x[1] = x[2] = x[3] = x[4] = x[5] = 0;

  // Observed points in Vector format
  Vector12f y0;
  y0 = getVectorFromPointCloud4(observed_correspondences);
  // Define a small variation that will be used to calculate the
  // effect of varyiing the different parameters of the pose
  double e = 0.00001;
  Eigen::Matrix< float, 3, 6 > jacobian = Eigen::Matrix< float, 3, 6 >::Zero();

  Vector6f pose_varied;
  Vector6f varied_vector;

  for(int i=0; i<200; i++)
  {
    // Where will the points of the model be with the given pose?
    // This will be the basis for our error calculation
    pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_model_pts (new pcl::PointCloud<pcl::PointXYZ>);
    // Calculate the model points w.r.t the current pose estimation
    pcl::transformPointCloud(*model_correspondences, *predicted_model_pts, getRotationMatrixFromPose(x) );
    Vector12f y = getVectorFromPointCloud4(predicted_model_pts);


    // Transform the points with a slight change of the parameters
    pcl::PointCloud<pcl::PointXYZ>::Ptr changed_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int idx=0;idx<6;idx++)
    {
      changed_pointcloud = transformVariedPose(model_correspondences, x, 0, e);
      Vector12f pts_after_variation = getVectorFromPointCloud4(changed_pointcloud);
      // ... and write these points in the jacobian column per column
      for(int j=0;j<12;j++)
      {
        jacobian(j,0) = pts_after_variation[j];
      }
    }



  }

  
  pcl::visualization::PCLVisualizer viewer;
  viewer.initCameraParameters ();
  viewer.addCoordinateSystem(0.3);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(observed_correspondences, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZ> (model_correspondences, "model_correspondences");
  viewer.addPointCloud<pcl::PointXYZ> (observed_correspondences, green_color, "observed_correspondences");
  viewer.addText("White pts = model correspondences, Green pts = observed correspondences", 5, 10 , "caption");
  viewer.spin();

  
  return 0;
}
