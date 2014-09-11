#include "suturo_perception_mbpe/correspondence_transform.h"

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
        // std::cout << c << std::endl;
        result->push_back(c);
      }
    }
  }
  pcl::transformPointCloud(*result, *result, getRotationMatrixFromPose(pose) );
  return result;
}

void CorrespondenceTransform::setCorrespondences(pcl::PointCloud<pcl::PointXYZ>::Ptr model_correspondences, pcl::PointCloud<pcl::PointXYZ>::Ptr observed_correspondences)
{
  model_correspondences_ = model_correspondences;
  observed_correspondences_ = observed_correspondences;
  y0_ = getVectorFromPointCloud4(observed_correspondences);
}

void CorrespondenceTransform::execute()
{

  // TODO Exception when model correspondences and observed_correspondences differ
  // TODO Exception when correspondence size < 3
 
  // Observed points in Vector format
  // Eigen::VectorXf y0;
  // y0 = ct.getVectorFromPointCloud4(observed_correspondences);

  // Define a small variation that will be used to calculate the
  // effect of varyiing the different parameters of the pose
  double e = 0.0001;
  // Eigen::Matrix< float, 12, 6 > jacobian = Eigen::Matrix< float, 12, 6 >::Zero();
  Eigen::MatrixXf jacobian(12,6); // correspondence_points * 3 x 6 pose parameters
  // int iterations;
  for(iterations_=0; iterations_<max_iterations_; iterations_++)
  {
    // Where will the points of the model be with the given pose?
    // This will be the basis for our error calculation
    pcl::PointCloud<pcl::PointXYZ>::Ptr predicted_model_pts (new pcl::PointCloud<pcl::PointXYZ>);
    // Calculate the model points w.r.t the current pose estimation
    pcl::transformPointCloud(*model_correspondences_, *predicted_model_pts, getRotationMatrixFromPose(x_) );
    Eigen::VectorXf y = getVectorFromPointCloud4(predicted_model_pts);


    // Transform the points with a slight change of the pose parameters
    pcl::PointCloud<pcl::PointXYZ>::Ptr changed_pointcloud (new pcl::PointCloud<pcl::PointXYZ>);
    for(int idx=0;idx<6;idx++)
    {
      changed_pointcloud = transformVariedPose(model_correspondences_, x_, idx, e);
      Eigen::VectorXf pts_after_variation = getVectorFromPointCloud4(changed_pointcloud);
      // ... and write these points in the jacobian column per column
      for(int j=0;j<12;j++)
      {
        jacobian(j,idx) = (pts_after_variation[j] - y[j]) / e;
      }
    }
    // Get the delta between the observed points
    // and the predicted points w.r.t to the current pose
    Eigen::VectorXf deltay = y0_ - y;
    std::cout << "Residual: " << deltay.norm() << std::endl;
    Eigen::Matrix< float, 6, 1 > deltax;
    Eigen::MatrixXf pinv = (jacobian.transpose() * jacobian).inverse() * jacobian.transpose();
    deltax = pinv * deltay;

    if( abs( (deltax.norm() / x_.norm()) < 1e-6 ) )
    {
      // change in transformation is nearly zero .... 
      break;
    }
    // Update pose estimate
    x_ = x_ + deltax;
  }
}
Eigen::Matrix< float, 6, 1 > CorrespondenceTransform::getEstimatedPose()
{
  return x_;
}

int CorrespondenceTransform::getIterationCount()
{
  return iterations_;
}
