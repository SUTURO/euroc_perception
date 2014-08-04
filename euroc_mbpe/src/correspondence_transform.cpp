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
#include <Eigen/Core>
#include <Eigen/SVD>

typedef Eigen::Matrix< float, 6, 1 > 	Vector6f;
typedef Eigen::Matrix< float, 12, 1 > 	Vector12f;

// template<typename _Matrix_Type_>
// _Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
// {
// 	Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
// 	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
// 	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
// }

// template<typename _Matrix_Type_>
// bool pseudoInverse(const _Matrix_Type_ &a, _Matrix_Type_ &result, double epsilon = std::numeric_limits<typename _Matrix_Type_::Scalar>::epsilon())
// {
//   if(a.rows()<a.cols())
//       return false;
// 
//   Eigen::JacobiSVD< _Matrix_Type_ > svd = a.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
// 
//   typename _Matrix_Type_::Scalar tolerance = epsilon * std::max(a.cols(), a.rows()) * svd.singularValues().array().abs().maxCoeff();
//   
//   result = svd.matrixV() * _Matrix_Type_( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().
//       array().inverse(), 0) ).asDiagonal() * svd.matrixU().adjoint();
// }
// 
Vector12f getVectorFromPointCloud4(pcl::PointCloud<pcl::PointXYZ>::Ptr p)
{
  if(p->points.size() != 4)
  {
    std::cout << "Encountered a pointcloud that's not of size 4 in getVectorFromPointCloud4" << std::endl;
    return Eigen::Matrix< float, 12, 1 >::Zero();
  }

  // for(int i = 0; i < p->points.size(); i++)
  // {
  //   std::cout <<  p->points[i].x << " " << p->points[i].y << " " << p->points[i].z << std::endl;
  // }
  Vector12f result = Eigen::Matrix< float, 12, 1 >::Zero();
  for(int i=0; i<4; i++)
  {
    result[0+(i*3)] = p->points[i].x;
    result[1+(i*3)] = p->points[i].y;
    result[2+(i*3)] = p->points[i].z;
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
  varied_vector[variation_idx] = e;
  Vector6f pose_varied = pose + varied_vector;    
  pcl::transformPointCloud(*cloud, *pts_param1, getRotationMatrixFromPose(pose_varied) );
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
  std::cout << "y0:" << std::endl << y0 << std::endl;
  // Define a small variation that will be used to calculate the
  // effect of varyiing the different parameters of the pose
  double e = 0.0001;
  Eigen::Matrix< float, 12, 6 > jacobian = Eigen::Matrix< float, 12, 6 >::Zero();
  int iterations;
  for(iterations=0; iterations<20; iterations++)
  {
    // Where will the points of the model be with the given pose?
    // This will be the basis for our error calculation
    pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_model_pts (new pcl::PointCloud<pcl::PointXYZ>);
    // Calculate the model points w.r.t the current pose estimation
    pcl::transformPointCloud(*model_correspondences, *predicted_model_pts, getRotationMatrixFromPose(x) );
    Eigen::Matrix< float, 12, 1 > y = getVectorFromPointCloud4(predicted_model_pts);
    std::cout << "y: " << std::endl << y << std::endl;


    // Transform the points with a slight change of the parameters
    pcl::PointCloud<pcl::PointXYZ>::Ptr changed_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int idx=0;idx<6;idx++)
    {
      changed_pointcloud = transformVariedPose(model_correspondences, x, idx, e);
      Vector12f pts_after_variation = getVectorFromPointCloud4(changed_pointcloud);
      // ... and write these points in the jacobian column per column
      for(int j=0;j<12;j++)
      {
        jacobian(j,idx) = (pts_after_variation[j] - y[j]) / e;
      }
    }
    // Get the delta between the observed points
    // and the predicted points w.r.t to the current pose
    Vector12f deltay = y0 - y;
    std::cout << "Residual: " << deltay.norm() << std::endl;
    std::cout << "deltay: " << std::endl << deltay << std::endl;
    // TODO CALCULATE THE (Moore-Penrose) pseudo inverse 
    // http://eigen.tuxfamily.org/index.php?title=FAQ#Is_there_a_method_to_compute_the_.28Moore-Penrose.29_pseudo_inverse_.3F 
    // std::cout << pseudoInverse(jacobian);
    Vector6f deltax;
    // std::cout << "Jacobian" << std::endl;
    // std::cout << jacobian << std::endl;
    // std::cout << "Jacobian^t" << std::endl;
    // std::cout << jacobian.transpose() << std::endl;
    // std::cout << "j^tj" << std::endl;
    // std::cout << jacobian.transpose() * jacobian << std::endl;
    // std::cout << "inv(j^t * j)" << std::endl;
    // std::cout << (jacobian.transpose() * jacobian).inverse() << std::endl;
    // std::cout << "inv(j^t * j) * J^t" << std::endl;
    // std::cout << (jacobian.transpose() * jacobian).inverse() * jacobian.transpose() << std::endl;
    // std::cout << "Should be nearly J" << std::endl;
    Eigen::Matrix<float,6,12> pinv = (jacobian.transpose() * jacobian).inverse() * jacobian.transpose();
    std::cout << (jacobian * pinv * jacobian) << std::endl;
    deltax = pinv * deltay;
    std::cout << "deltax: " << std::endl << deltax << std::endl;

    if( abs( (deltax.norm() / x.norm()) < 1e-6 ) )
    {
      // change in transformation is nearly zero .... 
      break;
    }
    // Update pose estimate
    x = x + deltax;
  }
  std::cout << "Final pose estimation after " << iterations << " iterations: " << std::endl << x << std::endl;

  
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
