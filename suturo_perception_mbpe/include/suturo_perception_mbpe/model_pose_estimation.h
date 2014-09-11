#ifndef MODEL_POSE_ESTIMATION_H
#define MODEL_POSE_ESTIMATION_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <suturo_msgs/Task.h>
#include <suturo_perception_cad_recognition/icp_fitter.h>
#include "perception_utils/logger.h"
#include <suturo_perception_mbpe/generate_pc_model.h>
#include <boost/algorithm/string.hpp>

/*
 * This class defines the interface for the perception pipeline.
 * It passes a set of suturo_msgs::Object to the constructor.
 * The Class will then create models from these representations 
 * online and will try to estimate the pose of a given PointCloud
 * against the given models.
 */
class ModelPoseEstimation {
public:
  // The objects are the Object specification from the parsed
  // YAML description of the EuRoC Task
  ModelPoseEstimation (boost::shared_ptr<std::vector<suturo_msgs::Object> > objects)
  {
    success_threshold_ = 1e-5;
    best_fit_model_ = 0;
    fitness_score_ = 0;
    pose_estimation_successful_ = false;
    objects_ = objects;
	  logger_ = suturo_perception::Logger("SuturoPerceptionMPE");
    generated_models_ = boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >(new std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> );
    voxel_size_ = 0;
  }

  //  This should be the observed pointcloud
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

  // If you want to do the pose estimation only for some of the models
  // (to reduce the search space), you can specify the ids here.
  // If you want to search for all models again, pass an empty vector
  void setModelsOfInterest(std::vector<int> ids);

  // The ICP process will return a fitness score that indicates how well
  // a given model fits to another cloud. You can threshold the return value
  // of ModelPoseEstimation::getPoseEstimationSuccessful with this value
  void setSuccessThreshold(double e);

  // Set the surface normal of the observed object. This is needed by the ICPFitter Library.
  void setSurfaceNormal(Eigen::Vector4f normal);

  // If the model AND the input cloud should be downsampled before the pose estimation
  // pass a value != 0 .
  void setVoxelSize(double size);

  void execute();

  /*
   *
   * The following methods are useful after execute() has been called
   *
   */
  

  // If the fitness score of a given model and the input cloud is below the treshold set by setSuccessThreshold, this method will return true
  // Available after execute()
  bool poseEstimationSuccessful();

  // Get the model id with the lowest fitness score 
  // Available after execute()
  int getBestFitModel();

  // Get the fitness score of the best match
  // Available after execute()
  double getFitnessScore();

  // Get the pose (x,y,z + quaternion) as a 7-dimensional vector.
  // The pose will be the one of the best matching model
  // Available after execute()
  Eigen::VectorXf getEstimatedPose();
private:
  /* data */
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
  std::vector<int> models_of_interest_;
  double success_threshold_;
  Eigen::Vector4f surface_normal_;
  int best_fit_model_;
  double fitness_score_;
  Eigen::VectorXf estimated_pose_;
  bool pose_estimation_successful_;
  double voxel_size_;
  boost::shared_ptr<std::vector<suturo_msgs::Object> > objects_;
  boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > generated_models_;
  suturo_perception::Logger logger_;
  GeneratePointCloudModel model_generator_;

  // Delete every model and create new ones
  void generateModels();


};
#endif
