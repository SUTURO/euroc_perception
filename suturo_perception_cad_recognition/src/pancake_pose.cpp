#include <suturo_perception_cad_recognition/pancake_pose.h>

Eigen::Matrix< float, 4, 4 > PancakePose::rotateAroundCrossProductOfNormals(
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

Cuboid PancakePose::computeCuboidFromBorderPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points)
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

void PancakePose::computeCuboidCornersWithMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points)
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

Eigen::Matrix< float, 4, 4> PancakePose::getTranslationMatrix(
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

// @override
pcl::PointCloud<pcl::PointXYZ>::Ptr PancakePose::execute()
{
  // Get the dimensions of the model
  // Rotate the model upwards, to get the proper dimensions
  _upwards_model = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _upwards_object = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _upwards_object_s1 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _upwards_object_s2 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _upwards_object_s3 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);

  Eigen::Matrix< float, 4, 4 > upwardRotationBox = 
    rotateAroundCrossProductOfNormals(Eigen::Vector3f(0,-1,0), Eigen::Vector3f(0,0,1));

  pcl::transformPointCloud (*_model_cloud, *_upwards_model, upwardRotationBox);
  // Store the first transformation of the model
  rotations_.push_back(upwardRotationBox);


  // Get the (square) dimensions with min max 3d
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points (new pcl::PointCloud<pcl::PointXYZRGB>);
  computeCuboidCornersWithMinMax3D(_upwards_model, corner_points);
  Cuboid c = 
    computeCuboidFromBorderPoints(corner_points);
  float model_height = c.length2;
  float model_width  = c.length1;
  float model_depth  = c.length3;
  std::cout << "Dimensions of the model: ";
  std::cout << model_height << " "; 
  std::cout << model_width << " "; 
  std::cout << model_depth << std::endl; 

  // Rotate the object and try to get the dimensions
  Eigen::Vector3f table_normal(
      _table_normal[0],
      _table_normal[1],
      _table_normal[2]);

  // Decide if the object is table normal is upside down or not. The table vector should point into the positive sector of the x-z plane
  Eigen::Vector3f rotation_base_vector;
  if(table_normal[1] < 0)
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
  //   rotateAroundCrossProductOfNormals(Eigen::Vector3f(0,1,0), table_normal);
  Eigen::Matrix< float, 4, 4 > transformationRotateObject = 
    rotateAroundCrossProductOfNormals(rotation_base_vector, table_normal);

  pcl::transformPointCloud (*_cloud_in, *_upwards_object, transformationRotateObject);   
  pcl::transformPointCloud (*_cloud_in, *_upwards_object_s1, transformationRotateObject);   
  Eigen::Vector4f input_cloud_centroid, rotated_input_cloud_centroid, 
    model_cloud_centroid, diff_of_centroids;
  pcl::compute3DCentroid(*_cloud_in, input_cloud_centroid); 
  pcl::compute3DCentroid(*_upwards_object, rotated_input_cloud_centroid); 
  diff_of_centroids = Eigen::Vector4f(0,0,0,0) - rotated_input_cloud_centroid;
  // Eigen::Affine3f transform = pcl::getTransformation(diff_of_centroids[0],
  //     diff_of_centroids[1], diff_of_centroids[2],0,0,0);
  Eigen::Matrix< float, 4, 4 > transform = 
    getTranslationMatrix(
        diff_of_centroids[0],
        diff_of_centroids[1],
        diff_of_centroids[2]);
  pcl::transformPointCloud(*_upwards_object, *_upwards_object_s2,transform);
  pcl::transformPointCloud(*_upwards_object, *_upwards_object,transform);

  // Estimate the dimensions of the object
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points_object (new pcl::PointCloud<pcl::PointXYZRGB>);
  computeCuboidCornersWithMinMax3D(_upwards_object, corner_points_object);
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
  // Eigen::Affine3f transformUpwards = pcl::getTransformation( 0,
  //     translate_upwards,0,0,0,0);
  Eigen::Matrix< float, 4, 4 > transformUpwards = 
    getTranslationMatrix(0,translate_upwards,0);
  pcl::transformPointCloud(*_upwards_object, *_upwards_object_s3, transformUpwards);
  pcl::transformPointCloud(*_upwards_object, *_upwards_object, transformUpwards);

  // Store the transposed matrix of the height-fitting transformation
  std::cout << transformUpwards << std::endl; 
  std::cout << transformUpwards.inverse() << std::endl;

  translations_.push_back(transformUpwards.inverse() ); 
  // Store the transposed matrix of the centroid alignment
  translations_.push_back(transform.inverse());
  // Store the transposed matrix of the rotation to fit the table normal
  rotations_.push_back(transformationRotateObject.transpose() ); 

  // Use ICP for the final alignment
  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputCloud(_upwards_object);
  icp.setInputTarget(_upwards_model);
  icp.setEuclideanFitnessEpsilon (0.000001f);
  // icp.setMaxCorrespondenceDistance (0.55);
  // icp.setRANSACOutlierRejectionThreshold(0.10f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  // boost::posix_time::ptime end = boost::posix_time::microsec_clock::local_time();
  _icp_transform = icp.getFinalTransformation();
  _icp_transform_inverse = icp.getFinalTransformation().inverse();


  // Rotate the model first
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix< float, 4, 4 > rotationBox = 
    rotateAroundCrossProductOfNormals(table_normal, Eigen::Vector3f(0,0,1));

  pcl::transformPointCloud (*_model_cloud, *transformed_cloud, rotationBox);   

  // Compute the centroids of both clouds and bring them closer together
  // for an rough initial alignment.
  pcl::compute3DCentroid(*transformed_cloud, model_cloud_centroid); 
  diff_of_centroids = input_cloud_centroid - model_cloud_centroid;
  Eigen::Affine3f transformC = pcl::getTransformation(diff_of_centroids[0],
      diff_of_centroids[1], diff_of_centroids[2],0,0,0);

  pcl::transformPointCloud(*transformed_cloud, *transformed_cloud,transformC);

  return transformed_cloud;
}

Eigen::Matrix<float, 4, 4>  PancakePose::getRotation()
{
  Eigen::Matrix<float, 4, 4> result;
  result.setIdentity();
  for (int i = 0; i < rotations_.size(); i++) {
    result *= rotations_.at(i);
  }
  return result;
}

Eigen::Matrix<float, 4, 4>  PancakePose::getTranslation()
{
  Eigen::Matrix<float, 4, 4> result = getTranslationMatrix(0,0,0);

  for (int i = 0; i < translations_.size(); i++) {
    float x,y,z, xr,yr,zr;
    x = translations_.at(i)(0,3);
    y = translations_.at(i)(1,3);
    z = translations_.at(i)(2,3);
    xr = result(0,3);
    yr = result(1,3);
    zr = result(2,3);
    result = getTranslationMatrix(x+xr, y+yr, z+zr);
  }
  return result;
}

Eigen::Matrix< float, 3, 3 > PancakePose::removeTranslationVectorFromMatrix(Eigen::Matrix<float,4,4> m)
{
  Eigen::Matrix< float, 3, 3 > result;
  result.setZero();
  result(0,0) = m(0,0);
  result(1,0) = m(1,0);
  result(2,0) = m(2,0);

  result(0,1) = m(0,1);
  result(1,1) = m(1,1);
  result(2,1) = m(2,1);

  result(0,2) = m(0,2);
  result(1,2) = m(1,2);
  result(2,2) = m(2,2);
  return result;
}

Eigen::Quaternionf PancakePose::getOrientation()
{
  Eigen::Matrix<float, 4, 4> final_transform =
    rotations_.at(1) * translations_.at(1) * translations_.at(0) * _icp_transform_inverse * rotations_.at(0);
  Eigen::Quaternionf q(removeTranslationVectorFromMatrix( final_transform ) );
  return q;
}

pcl::PointXYZ PancakePose::getOrigin()
{

  // Transform object center
  pcl::PointXYZ a(0,0,0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr origin (new pcl::PointCloud<pcl::PointXYZ>);
  origin->push_back(a);
  pcl::transformPointCloud(*origin, *origin, 
      rotations_.at(1) *
      translations_.at(1) * translations_.at(0) * _icp_transform_inverse * rotations_.at(0) );
  return origin->points.at(0);

}
