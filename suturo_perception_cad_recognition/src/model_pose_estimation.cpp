#include <suturo_perception_cad_recognition/model_pose_estimation.h>


void ModelPoseEstimation::setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  input_cloud_ = cloud;
}

void ModelPoseEstimation::setModelsOfInterest(std::vector<int> ids)
{
  models_of_interest_ = ids;
}
void ModelPoseEstimation::setSuccessThreshold(double e)
{
  success_threshold_ = e;
}

void ModelPoseEstimation::setSurfaceNormal(Eigen::Vector4f normal)
{
  surface_normal_ = normal;
}

void ModelPoseEstimation::setVoxelSize(double size)
{
  voxel_size_ = size;
}

bool   ModelPoseEstimation::poseEstimationSuccessful()
{
  if(getFitnessScore() < success_threshold_)
    return true;
  return false;
}

int    ModelPoseEstimation::getBestFitModel()
{
  return best_fit_model_;
}

double ModelPoseEstimation::getFitnessScore()
{
  return fitness_score_;
}

Eigen::VectorXf ModelPoseEstimation::getEstimatedPose()
{
  return estimated_pose_;
}

void ModelPoseEstimation::initForPipelineCall()
{
  // Default values - Better use the pipelineData now.
  this->setDumpICPFitterPointclouds(true); // Enable debugging. This will save pointclouds
  // to suturo_perception_cad_recognition/dumps
  // this->setVoxelSize(0.003f);
  this->setRemoveNaNs(true);


  // Use the parameters from dynamic reconfigure
  this->setVoxelSize(pipelineData_->mpeVoxelSize);
  this->setDumpICPFitterPointclouds(pipelineData_->mpeDumpICPFitterPointClouds); // Enable debugging. This will save pointclouds
  // to suturo_perception_cad_recognition/dumps
  max_icp_iterations_ = pipelineData_->mpeMaxICPIterations;
  success_threshold_ = pipelineData_->mpeSuccessThreshold;
  // mpeMaxICPIterations = 60;
  // mpeSuccessThreshold = 1e-5;
  // mpeVoxelSize = 0.003;
  // mpeDumpICPFitterPointClouds = false;
  
  std::vector<int> params = parseRequestArgs(pipelineData_->request_parameters_);
  logger_.logInfo((boost::format("request args count: %s") % params.size()).str());
  for (int i = 0; i < params.size(); i++)
  {
    ROS_INFO("MPE request param %d: %d", i, params[i]);
  }

  if(params.size() > 0)
  {
    std::cout << "Set models of interest to:";
    for (int i = 0; i < params.size(); i++)
    {
      std::cout << params[i] << " ";
    }
    std::cout << std::endl;
    setModelsOfInterest(params);
  }
}

void ModelPoseEstimation::execute()
{
  std::stringstream ss;
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  // Reset fitness score
  fitness_score_ = 999;

  // Set the default parameters if we are running inside our perception pipeline
  if(pipeline_mode_)
  {
    logger_.logInfo("pipeline mode on");
    initForPipelineCall();
  }
  else
  {
    logger_.logInfo("pipeline mode off");
  }

  generateModels();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud = input_cloud_;

  if(input_cloud == NULL)
  {
    logger_.logWarn("NULL pointcloud received. Skipping execution.");
    return;
  }
  if(input_cloud->points.size()==0)
  {
    logger_.logWarn("Empty pointcloud received. Skipping execution.");
    return;
  }

  if(remove_nans_)
  {
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input_cloud,*input_cloud, indices);
    ss << "removeNaNFromPointCloud removed " << indices.size() << "pts" << std::endl;
    logger_.logInfo(ss.str());

    ss.str("");
    ss << "Input Points after removal " << input_cloud->points.size() << "pts" << std::endl;
    logger_.logInfo(ss.str());
    ss.str("");
  }
  // Downsample the input cloud if desired
  if(voxel_size_ != 0)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    logger_.logInfo("Downsample obj");
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (input_cloud_);
    sor.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
    sor.filter (*voxel_cloud);
    input_cloud = voxel_cloud;
    // std::cout << "Downsampled Object Pointcloud has " << input_cloud->points.size() << "pts" << std::endl;
  }

  for (int i = 0; i < objects_->size(); i++) 
  {
    // Is the current index a "model of interest"?
    if(models_of_interest_.size()!=0 &&
        std::find(models_of_interest_.begin(), models_of_interest_.end(), i)==models_of_interest_.end())
    {
      logger_.logInfo("Skipping model on users demand");
      continue;
    }

    // Do the Pose estimation
    // ICPFitter fitter(input_cloud, generated_models_->at(i), surface_normal_);

    // Workaround since the Point Types mismatch. We could win a ms here if we can avoid that
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(*input_cloud, *input_cloud_xyz);
    copyPointCloud(*generated_models_->at(i), *model_cloud_xyz);
    // std::cout << "Copied Model Pointcloud has " << model_cloud_xyz->points.size() << "pts" << std::endl;
    // std::cout << "Copied Object Pointcloud has " << input_cloud_xyz->points.size() << "pts" << std::endl;
    // std::cout << "Using surface normal: " << surface_normal_ << std::endl;
    ICPFitter fitter(input_cloud_xyz, model_cloud_xyz, surface_normal_);
    fitter.rotateModelUp(false);
    fitter.setMaxICPIterations(max_icp_iterations_);

    boost::posix_time::ptime s_icp = boost::posix_time::microsec_clock::local_time();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr model_fitted = ria.execute();
    fitter.execute();
    boost::posix_time::ptime e_icp = boost::posix_time::microsec_clock::local_time();
    logger_.logTime(s_icp, e_icp, "Time for ICPFitter::execute()");

    if(fitter.getFitnessScore() < fitness_score_)
    {
      fitness_score_ = fitter.getFitnessScore();
      // Get the orientation of the aligned object.
      Eigen::Quaternionf orientation = fitter.getOrientation(); 
      // Get the origin of the aligned object.
      pcl::PointXYZ origin = fitter.getOrigin(); 
      // logger_.logInfo("pipeline_mode_ = true");
      suturo_msgs::Object &o = objects_->at(i);
      ss << "Model " << i << "("<< o.name <<") is below best fitness score. "; 
      ss << "Pose: " << orientation.x() << " " << orientation.y() << " " << orientation.z() << " " << orientation.w() << " " << origin << ". Score: " << fitter.getFitnessScore() << std::endl;
      logger_.logInfo(ss.str());
      ss.str("");

      estimated_pose_ = Eigen::VectorXf(7);
      estimated_pose_[0] = origin.x;
      estimated_pose_[1] = origin.y;
      estimated_pose_[2] = origin.z;

      estimated_pose_[3] = orientation.x();
      estimated_pose_[4] = orientation.y();
      estimated_pose_[5] = orientation.z();
      estimated_pose_[6] = orientation.w();

      best_fit_model_ = i;
    }

    // Dump the pointclouds that ICPFitter generated during it's execution
    if(dump_icp_fitter_pointclouds_)
      fitter.dumpPointClouds();

    // Workaround for a strange error ... The ICPFitter behaves
    // differently when you instantiate the standard ICP around it ...
    // Be careful when you comment this in .....
    // pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp;
    //
    //
   
  }

  // Store the result
  if(pipeline_mode_)
  {
    // Store results if we are running in the suturo_perception pipeline
    if(poseEstimationSuccessful())
    {

      // Get the informations from the best fit model

  // boost::shared_ptr<std::vector<suturo_msgs::Object> > objects_;





      boost::shared_ptr<moveit_msgs::CollisionObject> co(new moveit_msgs::CollisionObject);
      co->header.stamp = ros::Time::now();
      co->header.frame_id = "/tdepth_pcl";
      co->operation = moveit_msgs::CollisionObject::ADD;

      
      // TODO IMPLEMENT AND TEST THIS
      // Convert suturo_msg Object to CollisionObject
      suturo_msgs::Object &o = objects_->at(best_fit_model_);
      co->primitives.resize(o.primitives.size());
      co->primitive_poses.resize(o.primitive_poses.size());
      for (int i = 0; i < o.primitives.size(); i++)
      {
        // co->primitives[i].type = shape_msgs::SolidPrimitive::BOX;
        // co->primitives[i].type = o.primitives[i].type;
        // co->primitives[i].dimensions.resize ( o.primitives.dimensions.size() );
        co->primitives[i] = o.primitives[i];
        co->primitive_poses[i] = o.primitive_poses[i];
        // Has everything been copied properly?
        // std::cout << "CO Primitive" << co->primitives[i] << std::endl;
        // std::cout << "MPE Primitive" << o.primitives[i] << std::endl;
        // std::cout << "CO PrimitivePoses" << co->primitive_poses[i] << std::endl;
        // std::cout << "MPE PrimitivePoses" <<  o.primitive_poses[i] << std::endl;

        // // Copy the primitive pose by hand. This will not be handled by the SolidPrimitive Class.
        // co->primitive_poses[i].position.x = estimated_pose_[0];
        // co->primitive_poses[i].position.y = estimated_pose_[1];
        // co->primitive_poses[i].position.z = estimated_pose_[2];

        // Thanks to moritz 
        // Convert the original pose of every building block of the model
        // to the matched pose
        
        tf::Vector3 offset(o.primitive_poses[i].position.x,
            o.primitive_poses[i].position.y,
            o.primitive_poses[i].position.z);
        tf::Quaternion real_pose( estimated_pose_[3], estimated_pose_[4], estimated_pose_[5], estimated_pose_[6]);
        tf::Vector3 rotatedOffset = tf::quatRotate(real_pose, offset);
        co->primitive_poses[i].position.x = estimated_pose_[0] + rotatedOffset.x();
        co->primitive_poses[i].position.y = estimated_pose_[1] + rotatedOffset.y();
        co->primitive_poses[i].position.z = estimated_pose_[2] + rotatedOffset.z();

        // TODO WARNING. THIS WILL NOT WORK WITH ROTATED OBJECTS IN THE YAML.
        //      IMPLEMENT the handling ...
        co->primitive_poses[i].orientation.x = real_pose.x();
        co->primitive_poses[i].orientation.y = real_pose.y();
        co->primitive_poses[i].orientation.z = real_pose.z();
        co->primitive_poses[i].orientation.w = real_pose.w();
        


      }
      co->id = o.name;
      

      // ** CUBE ONLY ** 
      // co->primitives.resize(1);
      // co->primitives[0].type = shape_msgs::SolidPrimitive::BOX;
      // co->primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
      // co->primitive_poses.resize(1);
      // co->id = "box";

      // co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.05;
      // co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.05;
      // co->primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
      // co->primitive_poses[0].position.x = estimated_pose_[0];
      // co->primitive_poses[0].position.y = estimated_pose_[1];
      // co->primitive_poses[0].position.z = estimated_pose_[2];
      // geometry_msgs::Quaternion q;
      // q.x = estimated_pose_[3];
      // q.y = estimated_pose_[4];
      // q.z = estimated_pose_[5]; 
      // q.w = estimated_pose_[6];
      // co->primitive_poses[0].orientation = q;

      pipelineObject_->set_c_mpe_object(co);
      pipelineObject_->set_c_mpe_success(true);
    }
    else
    {
      pipelineObject_->set_c_mpe_success(false);
    }

  }
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger_.logTime(s, e, "Time for execute()");
}

void ModelPoseEstimation::generateModels()
{
  if(objects_->size() == 0)
  {
    logger_.logWarn("The object count from the task description is 0. Can't generate models");
    return;
  }
  generated_models_->clear();  
  for (int i = 0; i < objects_->size(); i++) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr shape = model_generator_.generateComposed(objects_->at(i).primitives, objects_->at(i).primitive_poses);

    if(voxel_size_ != 0)
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxel_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
      logger_.logInfo("Downsample model");
      pcl::VoxelGrid<pcl::PointXYZRGB> sor;
      sor.setInputCloud (shape);
      sor.setLeafSize (voxel_size_, voxel_size_, voxel_size_);
      sor.filter (*voxel_cloud);
      generated_models_->push_back( voxel_cloud );
      std::cout << "Generated Pointcloud with " << voxel_cloud->points.size() << "pts" << std::endl;
    }
    else
    {
      generated_models_->push_back( shape );
      std::cout << "Generated Pointcloud with " << shape->points.size() << "pts" << std::endl;
    }
  }
}


void ModelPoseEstimation::setDumpICPFitterPointclouds(bool b)
{
  dump_icp_fitter_pointclouds_ = b;
}

boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > ModelPoseEstimation::getGeneratedModels()
{
  return generated_models_;
}

std::string ModelPoseEstimation::getName()
{
  return "ModelPoseEstimation";
}


void ModelPoseEstimation::setRemoveNaNs(bool b)
{
  remove_nans_ = b;
}


std::vector<int> ModelPoseEstimation::parseRequestArgs(std::string req)
{
  std::vector<std::string> rets;
  std::vector<int> ret;
  std::string delims = ",()";
  std::string tmp;
  bool isMPE = false;
  for (int i = 0; i < req.size(); i++)
  {
    bool isDelim = false;
    for (int j = 0; j < delims.size(); j++)
    {
      if (req[i] == delims[j])
        isDelim = true;
    }
    if (isDelim)
    {
      if (isMPE)
      {
        rets.push_back(tmp);
        //logger_.logInfo((boost::format("mpe part: %s") % tmp.c_str()).str());
        try
        {
          ret.push_back(boost::lexical_cast<int>(tmp));
        }
        catch (...)
        {
        }
      }
      if (tmp.compare(getName()) == 0)
      {
        isMPE = true;
      }
      //ret.push_back(tmp);
      tmp = "";
    }
    else
    {
      tmp += req[i];
    }
    if (req[i] == ')')
      isMPE = false;
  }
  //ret.push_back(tmp);
  
  return ret;
}

