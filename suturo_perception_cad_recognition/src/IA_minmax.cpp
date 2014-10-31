#include <suturo_perception_cad_recognition/IA_minmax.h>

using namespace suturo_perception;

Eigen::Matrix<float, 4, 4> IAMinMax::getTransformations()
{

  Eigen::Matrix<float, 4, 4> final_transform =
    rotations_.at(0) * translations_.at(0);
    // Eigen::Matrix<float, 4, 4>::Identity();
  return final_transform;
}
// pcl::PointXYZ IACentroid::getOrigin()
// {
//   // Transform object center
//   pcl::PointXYZ a(0,0,0);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr origin (new pcl::PointCloud<pcl::PointXYZ>);
//   origin->push_back(a);
//   pcl::transformPointCloud(*origin, *origin, 
//       rotations_.at(0) *
//       translations_.at(1) * translations_.at(0) );
// 
//   // std::cout << "In class transformations:" << std::endl;
//   // std::cout << rotations_.at(0) << std::endl;
//   // std::cout << translations_.at(1) << std::endl;
//   // std::cout << translations_.at(0) << std::endl;
//   // std::cout << "----- END ----" << std::endl;
//   return origin->points.at(0);
// }

void IAMinMax::execute()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_s1 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_s2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr result_s3 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_min_pt_object (new pcl::PointCloud<pcl::PointXYZ>);
  std::cout << "Doing the minmax IA method" << std::endl;

  // Create a bogus point so pcd_write doesnt throw an exception for now.
  // TODO: remove
  // pcl::PointXYZ a(0,0,0);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr origin (new pcl::PointCloud<pcl::PointXYZ>);
  // result->push_back(a);

  // TODO Minmax before or after table normal alignment?

  pcl::PointXYZ min_pt_model, max_pt_model, min_pt_object, max_pt_object;
  pcl::getMinMax3D(*_model_cloud, min_pt_model, max_pt_model);
  pcl::getMinMax3D(*_cloud_in, min_pt_object, max_pt_object);

  // Add these points for debugging
  result_s3->push_back(min_pt_model);
  result_s3->push_back(max_pt_model);
  result_s3->push_back(min_pt_object);
  result_s3->push_back(max_pt_object);
  transformed_min_pt_object->push_back(min_pt_object);
  transformed_min_pt_object->push_back(max_pt_object);

  // Convert the points to eigen vectors
  Eigen::Vector3f v_min_pt_model = min_pt_model.getVector3fMap();
  Eigen::Vector3f v_max_pt_model = max_pt_model.getVector3fMap();
  Eigen::Vector3f v_min_pt_object = min_pt_object.getVector3fMap();
  Eigen::Vector3f v_max_pt_object = max_pt_object.getVector3fMap();
  // Calculate the diagonal through the min/max points (e.g. through the object)
  Eigen::Vector3f diagonal_model  = v_max_pt_model -  v_min_pt_model;
  Eigen::Vector3f diagonal_object = v_max_pt_object - v_min_pt_object;

  // Step 1
  // Translate the object to the model by the min_pts of both.
  Eigen::Vector3f diff_of_min_pts = v_min_pt_model - v_min_pt_object;
  
  Eigen::Matrix< float, 4, 4 > transform_min_pt = 
    getTranslationMatrix(
        diff_of_min_pts[0],
        diff_of_min_pts[1],
        diff_of_min_pts[2]);

  pcl::transformPointCloud(*_cloud_in, *result_s1,transform_min_pt);
  pcl::transformPointCloud(*_cloud_in, *_result  ,transform_min_pt);
  pcl::transformPointCloud(*transformed_min_pt_object, *transformed_min_pt_object ,transform_min_pt);
  translations_.push_back(transform_min_pt.inverse() ); 
  _object_transformation_steps.push_back(result_s1);

  // Step 2
  // Rotate the diagonal of the object into the diagonal of the
  // model

  // TODO: Is it safe to always use *-1 on diagonal_object? 
  // Maybe it's better to calculate the diagonal first
  Eigen::Matrix< float, 4, 4 > transformationRotateDiagonal = 
    rotateAroundCrossProductOfNormals(diagonal_model, diagonal_object * -1);
  pcl::transformPointCloud(*_result, *result_s2,transformationRotateDiagonal);
  pcl::transformPointCloud(*_result, *_result  ,transformationRotateDiagonal);
  pcl::transformPointCloud(*transformed_min_pt_object, *transformed_min_pt_object ,transformationRotateDiagonal);
  rotations_.push_back(transform_min_pt.inverse() ); 
  _object_transformation_steps.push_back(result_s2);

  // DEBUGGING 
  _object_transformation_steps.push_back(result_s3);
  // Add transformed min_pt_model
  _object_transformation_steps.push_back(transformed_min_pt_object);
  /*
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
  Eigen::Vector4f rotated_input_cloud_centroid, model_cloud_centroid,
    diff_of_centroids;
  pcl::compute3DCentroid(*_result, rotated_input_cloud_centroid); 
  // diff_of_centroids = Eigen::Vector4f(0,0,0,0) - rotated_input_cloud_centroid;
  // else
  // {
    pcl::compute3DCentroid(*_model_cloud, model_cloud_centroid); 
    diff_of_centroids = model_cloud_centroid - rotated_input_cloud_centroid;
    std::cout << "ModelCentroid is at " << model_cloud_centroid << "." << "Diff of centroids:" << diff_of_centroids;
  // }
  Eigen::Matrix< float, 4, 4 > transform = 
    getTranslationMatrix(
        diff_of_centroids[0],
        diff_of_centroids[1],
        diff_of_centroids[2]);
  // std::cout << "Resulting centroid shift transform" << transform << std::endl;
  // std::cout << "Resulting centroid shift transform INVERSE" << transform.inverse() << std::endl;
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
  // std::cout << "Translating upwards by " << translate_upwards * 100 << "=" << model_height * 100 << "-" << object_height * 100 << std::endl;
  Eigen::Matrix< float, 4, 4 > transformUpwards = 
    getTranslationMatrix(0,translate_upwards,0);
  pcl::transformPointCloud(*_result, *result_s3, transformUpwards);
  pcl::transformPointCloud(*_result, *_result, transformUpwards);
  _object_transformation_steps.push_back(result_s3);

  // Store the transposed matrix of the height-fitting transformation
  translations_.push_back(transformUpwards.inverse() ); 
  // Store the transposed matrix of the centroid alignment
  translations_.push_back(transform.inverse());

  // std::cout << "Showing translations in class: " << std::endl;
  // for (int i = 0; i < translations_.size(); i++) {
  //   std::cout << "idx: " << i << " " << translations_.at(i) << std::endl;
  // }
  */

}

