#include <suturo_perception_cad_recognition/IA_centroid.h>

using namespace suturo_perception;

void IACentroid::execute()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_s1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_s2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_s3 (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Doing the classic IA method" << std::endl;

  // Step 1: Rotate the object with the table normal
  Eigen::Vector3f table_normal(
      _table_normal[0],
      _table_normal[1],
      _table_normal[2]);
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

  Eigen::Matrix< float, 4, 4 > transformationRotateObject = 
    rotateAroundCrossProductOfNormals(rotation_base_vector, table_normal);
  std::cout << "LOL"<< std::endl;
  pcl::transformPointCloud (*_cloud_in, *_result, transformationRotateObject);   
  std::cout << "LOL2"<< std::endl;
  pcl::transformPointCloud (*_cloud_in, *result_s1, transformationRotateObject);   
  _object_transformation_steps.push_back(result_s1);
  // Store the rotation done
  rotations_.push_back(transformationRotateObject.transpose() ); 


  // Step 2: Move the rotated object cloud to the origin of the coordinate system
  std::cout << "Step2 in IACentroid" << std::endl;
  Eigen::Vector4f rotated_input_cloud_centroid, 
    diff_of_centroids;
  pcl::compute3DCentroid(*_result, rotated_input_cloud_centroid); 
  diff_of_centroids = Eigen::Vector4f(0,0,0,0) - rotated_input_cloud_centroid;
  Eigen::Matrix< float, 4, 4 > transform = 
    getTranslationMatrix(
        diff_of_centroids[0],
        diff_of_centroids[1],
        diff_of_centroids[2]);
  pcl::transformPointCloud(*_result, *result_s2,transform);
  pcl::transformPointCloud(*_result, *_result,transform);
  _object_transformation_steps.push_back(result_s2);

  // Step 3: Calculate the height of the model and the object, and align
  // the top of the object with the top of the model
  // Get the (square) dimensions with min max 3d
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points (new pcl::PointCloud<pcl::PointXYZRGB>);
  computeCuboidCornersWithMinMax3D(_model_cloud, corner_points);
  Cuboid c = 
    computeCuboidFromBorderPoints(corner_points);
  float model_height = c.length2;
  float model_width  = c.length1;
  float model_depth  = c.length3;
  std::cout << "Dimensions of the model: ";
  std::cout << model_height << " "; 
  std::cout << model_width << " "; 
  std::cout << model_depth << std::endl; 

  // Estimate the dimensions of the object
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points_object (new pcl::PointCloud<pcl::PointXYZRGB>);
  computeCuboidCornersWithMinMax3D(_result, corner_points_object);
  Cuboid oc = 
    computeCuboidFromBorderPoints(corner_points_object);
  float object_height = oc.length2;
  float object_width  = oc.length1;
  float object_depth  = oc.length3;
  std::cout << "Dimensions of the object: ";
  std::cout << object_height << " "; 
  std::cout << object_width << " "; 
  std::cout << object_depth << std::endl; 

  // Translate the object to align it with the top of the model
  float translate_upwards = model_height - object_height;
  Eigen::Matrix< float, 4, 4 > transformUpwards = 
    getTranslationMatrix(0,translate_upwards,0);
  pcl::transformPointCloud(*_result, *result_s3, transformUpwards);
  pcl::transformPointCloud(*_result, *_result, transformUpwards);
  _object_transformation_steps.push_back(result_s3);

  // Store the transposed matrix of the height-fitting transformation
  translations_.push_back(transformUpwards.inverse() ); 
  // Store the transposed matrix of the centroid alignment
  translations_.push_back(transform.inverse());
}

