#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <boost/program_options.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SVD>
#include "perception_utils/logger.h"

typedef Eigen::Matrix< float, 6, 1 > 	Vector6f;
typedef Eigen::Matrix< float, 12, 1 > 	Vector12f;


// Vector12f getVectorFromPointCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr p)
Eigen::VectorXf getVectorFromPointCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr p)
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

Eigen::Matrix4f getRotationMatrixFromPose(Eigen::Matrix< float, 6, 1 > pose)
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
    Eigen::Matrix< float, 6, 1 > pose, int variation_idx, float e)
{
  // Transform the points with a slight change of param1 ...
  pcl::PointCloud<pcl::PointXYZ>::Ptr pts_param1 (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix< float, 6, 1 > varied_vector = Vector6f::Zero();
  varied_vector[variation_idx] = e;
  Eigen::Matrix< float, 6, 1 > pose_varied = pose + varied_vector;    
  pcl::transformPointCloud(*cloud, *pts_param1, getRotationMatrixFromPose(pose_varied) );
  return pts_param1;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generateBoxModelFromPose(Eigen::Matrix< float, 6, 1 > pose){
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

  pcl::transformPointCloud(*result, *result, getRotationMatrixFromPose(pose) );
  return result;
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
  d[2] = M_PI/4; // Rotate pi/4 around the z-axis
  d[3] = 0.5; // Translation on the x axis
  Eigen::Matrix4f transform_1 = getRotationMatrixFromPose(d);
  pcl::transformPointCloud(*model_correspondences, *observed_correspondences, transform_1 );


  // Start time measuring ...
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();

  // Initial guess for the pose
  Eigen::Matrix< float, 6, 1 > x;
  x[0] = x[1] = x[2] = x[3] = x[4] = x[5] = 0;

  // Observed points in Vector format
  Eigen::VectorXf y0;
  y0 = getVectorFromPointCloud4(observed_correspondences);

  // Define a small variation that will be used to calculate the
  // effect of varyiing the different parameters of the pose
  double e = 0.0001;
  // Eigen::Matrix< float, 12, 6 > jacobian = Eigen::Matrix< float, 12, 6 >::Zero();
  Eigen::MatrixXf jacobian(12,6); // correspondence_points * 3 x 6 pose parameters
  int iterations;
  for(iterations=0; iterations<20; iterations++)
  {
    // Where will the points of the model be with the given pose?
    // This will be the basis for our error calculation
    pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_model_pts (new pcl::PointCloud<pcl::PointXYZ>);
    // Calculate the model points w.r.t the current pose estimation
    pcl::transformPointCloud(*model_correspondences, *predicted_model_pts, getRotationMatrixFromPose(x) );
    Eigen::VectorXf y = getVectorFromPointCloud4(predicted_model_pts);


    // Transform the points with a slight change of the pose parameters
    pcl::PointCloud<pcl::PointXYZ>::Ptr changed_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int idx=0;idx<6;idx++)
    {
      changed_pointcloud = transformVariedPose(model_correspondences, x, idx, e);
      Eigen::VectorXf pts_after_variation = getVectorFromPointCloud4(changed_pointcloud);
      // ... and write these points in the jacobian column per column
      for(int j=0;j<12;j++)
      {
        jacobian(j,idx) = (pts_after_variation[j] - y[j]) / e;
      }
    }
    // Get the delta between the observed points
    // and the predicted points w.r.t to the current pose
    Eigen::VectorXf deltay = y0 - y;
    std::cout << "Residual: " << deltay.norm() << std::endl;
    Eigen::Matrix< float, 6, 1 > deltax;
    Eigen::MatrixXf pinv = (jacobian.transpose() * jacobian).inverse() * jacobian.transpose();
    deltax = pinv * deltay;

    if( abs( (deltax.norm() / x.norm()) < 1e-6 ) )
    {
      // change in transformation is nearly zero .... 
      break;
    }
    // Update pose estimate
    x = x + deltax;
  }
  std::cout << "Final pose estimation after " << iterations << " iterations: " << std::endl << x << std::endl;
  boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  suturo_perception::Logger logger("correspondence_transform");
  logger.logTime(s, end, "pose estimation");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

  cloud->points.resize (5);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = i; 
    cloud->points[i].y = i / 2; 
    cloud->points[i].z = 0;
  }

  // Start the visualizer
  pcl::visualization::PCLVisualizer p ("test_shapes");
  p.addCoordinateSystem (0.3);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(observed_correspondences, 0, 255, 0);
  p.addPointCloud<pcl::PointXYZ> (model_correspondences, "model_correspondences");
  p.addPointCloud<pcl::PointXYZ> (observed_correspondences, green_color, "observed_correspondences");
  p.addText("White pts = model correspondences, Green pts = observed correspondences, Green box = The model fitted to the observed points, Yellow pts = line between correspondences", 5, 10 , "caption");

  // Create a model of the box as a Polygon
  pcl::PointCloud<pcl::PointXYZ>::Ptr box_model = generateBoxModelFromPose(x);
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[0], box_model->points[1], 0.0, 1.0, 0.0,"line1");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[1], box_model->points[3], 0.0, 1.0, 0.0,"line2");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[1], box_model->points[5], 0.0, 1.0, 0.0,"line3");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[0], box_model->points[2], 0.0, 1.0, 0.0,"line4");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[0], box_model->points[4], 0.0, 1.0, 0.0,"line5");

  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[2], box_model->points[3], 0.0, 1.0, 0.0,"line6");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[4], box_model->points[5], 0.0, 1.0, 0.0,"line7");

  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[6], box_model->points[7], 0.0, 1.0, 0.0,"line8");

  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[6], box_model->points[2], 0.0, 1.0, 0.0,"line9");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[6], box_model->points[4], 0.0, 1.0, 0.0,"line10");

  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[7], box_model->points[3], 0.0, 1.0, 0.0,"line11");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (box_model->points[7], box_model->points[5], 0.0, 1.0, 0.0,"line12");

  // Add Correspondences
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_correspondences->points[0], observed_correspondences->points[0], 1.0, 1.0, 0.0,"corrline1");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_correspondences->points[1], observed_correspondences->points[1], 1.0, 1.0, 0.0,"corrline2");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_correspondences->points[2], observed_correspondences->points[2], 1.0, 1.0, 0.0,"corrline3");
  p.addLine<pcl::PointXYZ, pcl::PointXYZ> (model_correspondences->points[3], observed_correspondences->points[3], 1.0, 1.0, 0.0,"corrline4");

  p.spin();
  
  return 0;
}
