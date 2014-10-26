#include <suturo_perception_cad_recognition/IA_centroid.h>

using namespace suturo_perception;

void IACentroid::execute()
{
  std::cout << "Doing the classic IA method" << std::endl;

  // Step 1: Rotate the object with the table normal
  Eigen::Vector3f rotation_base_vector;
  if(_table_normal[1] < 0)
  {
    // Table Normal Vector is pointing down. Invert the Y Unit vector
    rotation_base_vector[0] = rotation_base_vector[2] = 0;
    rotation_base_vector[1] = -1;
  }
  else
  {
    // Table Normal Vector is pointing up. No need to invert the Y Unit vector
    rotation_base_vector[0] = rotation_base_vector[2] = 0;
    rotation_base_vector[1] = 1;
  }

  // Eigen::Matrix< float, 4, 4 > transformationRotateObject = 
  //   rotateAroundCrossProductOfNormals(rotation_base_vector, _table_normal);
  // pcl::transformPointCloud (*_cloud_in, *_result, transformationRotateObject);   
  // pcl::transformPointCloud (*_cloud_in, *_result_s1, transformationRotateObject);   
  // _object_transformation_steps.push_back(_upwards_object_s1);
  // // Store the rotation done
  // rotations_.push_back(transformationRotateObject.transpose() ); 


  // // Step 2: Move the rotated object cloud to the origin of the coordinate system
  // Eigen::Vector4f rotated_input_cloud_centroid, 
  //   model_cloud_centroid, diff_of_centroids;
  // pcl::compute3DCentroid(*_upwards_object, rotated_input_cloud_centroid); 
  // diff_of_centroids = Eigen::Vector4f(0,0,0,0) - rotated_input_cloud_centroid;
  // Eigen::Matrix< float, 4, 4 > transform = 
  //   getTranslationMatrix(
  //       diff_of_centroids[0],
  //       diff_of_centroids[1],
  //       diff_of_centroids[2]);
  // pcl::transformPointCloud(*_upwards_object, *_upwards_object_s2,transform);
  // pcl::transformPointCloud(*_upwards_object, *_upwards_object,transform);
  // _object_transformation_steps.push_back(_upwards_object_s2);
}

