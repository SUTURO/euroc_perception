#include <suturo_perception_cad_recognition/icp_fitter.h>

Eigen::Matrix< float, 4, 4 > ICPFitter::rotateAroundCrossProductOfNormals(
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

Cuboid ICPFitter::computeCuboidFromBorderPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points)
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

void ICPFitter::computeCuboidCornersWithMinMax3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr corner_points)
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

Eigen::Matrix< float, 4, 4> ICPFitter::getTranslationMatrix(
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
pcl::PointCloud<pcl::PointXYZ>::Ptr ICPFitter::execute()
{
  // Get the dimensions of the model
  // Rotate the model upwards, to get the proper dimensions
  _upwards_model = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _upwards_object = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _upwards_object_s1 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _upwards_object_s2 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _upwards_object_s3 = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  _icp_fitted_object = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
  std::stringstream ss;
  ss << "Amount of points in the input cloud: ";
  ss << _cloud_in->points.size() << std::endl; 
  logger_.logInfo(ss.str());
  // Hold a pointer to every pointcloud, that represents a step in the
  // fitting process
  _model_transformation_steps.clear();
  _object_transformation_steps.clear();
  // Add the initial pointclouds
  _model_transformation_steps.push_back(_model_cloud);
  _object_transformation_steps.push_back(_cloud_in);

  // If desired, perform an upward rotation of the model first
  if(_rotate_model_upwards)
  {
    Eigen::Matrix< float, 4, 4 > upwardRotationBox = 
      rotateAroundCrossProductOfNormals(Eigen::Vector3f(0,-1,0), Eigen::Vector3f(0,0,1));

    pcl::transformPointCloud (*_model_cloud, *_upwards_model, upwardRotationBox);
    _model_transformation_steps.push_back(_upwards_model);
    // Store the first transformation of the model
    rotations_.push_back(upwardRotationBox);
  }
  else
  {
    pcl::copyPointCloud (*_model_cloud, *_upwards_model);
    // Add an identity transformation
    Eigen::Matrix< float, 4, 4 > upwardRotationBox = Eigen::Matrix<float,4,4>::Identity();
    rotations_.push_back(upwardRotationBox);
    _model_transformation_steps.push_back(_upwards_model);
  }

  // Prepare and execute initial alignment
  _initial_alignment->setModelCloud(_upwards_model);
  _initial_alignment->execute();
  std::cout << "IA::execute() done" << std::endl;

  // Copy results from IA execution
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> ia_obj_transformations = _initial_alignment->getObjectTransformationSteps();
  _object_transformation_steps.insert(_object_transformation_steps.end(),
      ia_obj_transformations.begin(),
      ia_obj_transformations.end());
  std::cout << "transformation pcs copied" << std::endl;

  translations_ = _initial_alignment->getTranslations();
  std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > ia_rotations =  _initial_alignment->getRotations();
  rotations_.insert( rotations_.end(),
      ia_rotations.begin(), ia_rotations.end());

  _upwards_object = _initial_alignment->getResult();
  std::cout << "IA done" << std::endl;
  

  // Use ICP for the final alignment, after the object has been initially aligned.
  pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(_upwards_object);
  icp.setInputTarget(_upwards_model);
  if(_max_icp_iterations != NO_ICP_MAX_ITERATIONS)
    icp.setMaximumIterations(_max_icp_iterations);

  // icp.setEuclideanFitnessEpsilon (0.000001f);
  if(_max_icp_distance != NO_ICP_DISTANCE)
    icp.setMaxCorrespondenceDistance (_max_icp_distance);

  // icp.setRANSACOutlierRejectionThreshold(0.10f);
  pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
  icp.align(*Final);
  pcl::copyPointCloud(*Final, *_icp_fitted_object);
  _object_transformation_steps.push_back(_icp_fitted_object);
  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  _icp_fitness_score = icp.getFitnessScore();
  std::cout << icp.getFinalTransformation() << std::endl;
  _icp_transform = icp.getFinalTransformation();
  _icp_transform_inverse = icp.getFinalTransformation().inverse();


  // Rotate the model first
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
  //      WARN: The following code will break the cad_recognition visualization.
  //      Nevertheless, we don't need it for
  //      our newer software modules
  //
  // Eigen::Matrix< float, 4, 4 > rotationBox = 
  //   rotateAroundCrossProductOfNormals(table_normal, Eigen::Vector3f(0,0,1));

  // pcl::transformPointCloud (*_model_cloud, *transformed_cloud, rotationBox);   

  // // Compute the centroids of both clouds and bring them closer together
  // // for an rough initial alignment.
  // pcl::compute3DCentroid(*transformed_cloud, model_cloud_centroid); 
  // diff_of_centroids = input_cloud_centroid - model_cloud_centroid;
  // Eigen::Affine3f transformC = pcl::getTransformation(diff_of_centroids[0],
  //     diff_of_centroids[1], diff_of_centroids[2],0,0,0);

  // pcl::transformPointCloud(*transformed_cloud, *transformed_cloud,transformC);

  return transformed_cloud;
}

Eigen::Matrix<float, 4, 4>  ICPFitter::getRotation()
{
  Eigen::Matrix<float, 4, 4> result;
  result.setIdentity();
  for (int i = 0; i < rotations_.size(); i++) {
    result *= rotations_.at(i);
  }
  return result;
}

Eigen::Matrix<float, 4, 4>  ICPFitter::getTranslation()
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

Eigen::Matrix< float, 3, 3 > ICPFitter::removeTranslationVectorFromMatrix(Eigen::Matrix<float,4,4> m)
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

Eigen::Quaternionf ICPFitter::getOrientation()
{
  Eigen::Matrix<float, 4, 4> final_transform =
    _initial_alignment->getTransformations() * _icp_transform_inverse * rotations_.at(0);
  Eigen::Quaternionf q(removeTranslationVectorFromMatrix( final_transform ) );
  return q;
}

pcl::PointXYZ ICPFitter::getOrigin()
{
  // Transform object center
  pcl::PointXYZ a(0,0,0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr origin (new pcl::PointCloud<pcl::PointXYZ>);
  origin->push_back(a);
  pcl::transformPointCloud(*origin, *origin, 
       _initial_alignment->getTransformations() * 
_icp_transform_inverse * rotations_.at(0) );
  return origin->points.at(0); 

  return origin->points.at(0);

}

void ICPFitter::setMaxICPIterations(int v)
{
  _max_icp_iterations = v;
}

double ICPFitter::getFitnessScore()
{
  return _icp_fitness_score;
}

void ICPFitter::setMaxCorrespondenceDistance(double v)
{
  _max_icp_distance = v;
}

void ICPFitter::rotateModelUp(bool m)
{
  _rotate_model_upwards = m;
}

void ICPFitter::dumpPointClouds()
{
  // Prefix the files with the current timestamp
  std::string package_path = ros::package::getPath("suturo_perception_cad_recognition");
  std::string dump_folder = "/dumps/";
  std::time_t t = std::time(0);

  for (int i = 0; i < _object_transformation_steps.size(); i++) {
    // write pcd
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << package_path << dump_folder << t << "_object_s_"<< i << ".pcd";
    writer.write(ss.str(), *_object_transformation_steps.at(i));
    // std::cerr << "Saved " << output_cloud_->points.size () << " data points" << std::endl;
  }

  for (int i = 0; i < _model_transformation_steps.size(); i++) {
    // write pcd
    pcl::PCDWriter writer;
    std::stringstream ss;
    ss << package_path << dump_folder << t << "_model_s_"<< i << ".pcd";
    writer.write(ss.str(), *_model_transformation_steps.at(i));
    // std::cerr << "Saved " << output_cloud_->points.size () << " data points" << std::endl;
  }
}


void ICPFitter::setCalculateModelCentroid(bool b)
{
  _calc_model_centroid = b;
}


void ICPFitter::setIAMethod(ICPFitter::IAMethod m)
{
  if(m == ICPFitter::IA_CENTROID)
  {
      _initial_alignment = boost::shared_ptr<suturo_perception::IAMethod> (new suturo_perception::IACentroid(_cloud_in, _model_cloud, _table_normal));
  }
  else if(m == ICPFitter::IA_MINMAX)
  {
      _initial_alignment = boost::shared_ptr<suturo_perception::IAMethod> (new suturo_perception::IAMinMax(_cloud_in, _model_cloud, _table_normal));
  }
}
