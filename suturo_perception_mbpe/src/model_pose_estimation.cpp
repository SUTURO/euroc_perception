#include <suturo_perception_mbpe/model_pose_estimation.h>


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

void ModelPoseEstimation::execute()
{

  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  // Reset fitness score
  fitness_score_ = 999;

  generateModels();
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud = input_cloud_;
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
    std::cout << "Downsampled Object Pointcloud has " << input_cloud->points.size() << "pts" << std::endl;
  }

  for (int i = 0; i < objects_->size(); i++) {
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
    std::cout << "Copied Model Pointcloud has " << model_cloud_xyz->points.size() << "pts" << std::endl;
    std::cout << "Copied Object Pointcloud has " << input_cloud_xyz->points.size() << "pts" << std::endl;
    std::cout << "Using surface normal: " << surface_normal_ << std::endl;
    ICPFitter fitter(input_cloud_xyz, model_cloud_xyz, surface_normal_);
    fitter.rotateModelUp(false);
    fitter.setMaxICPIterations(60);

    boost::posix_time::ptime s_icp = boost::posix_time::microsec_clock::local_time();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr model_fitted = ria.execute();
    fitter.execute();
    boost::posix_time::ptime e_icp = boost::posix_time::microsec_clock::local_time();
    logger_.logTime(s_icp, e_icp, "Time for ICPFitter::execute()");

    if(fitter.getFitnessScore() < fitness_score_)
      fitness_score_ = fitter.getFitnessScore();

    // Dump the pointclouds that ICPFitter generated during it's execution
    // TODO: check bool for activation
    fitter.dumpPointClouds();
   
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
