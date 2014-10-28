#ifndef MODEL_POSE_ESTIMATION_H
#define MODEL_POSE_ESTIMATION_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <suturo_msgs/Task.h>
#include <suturo_perception_cad_recognition/icp_fitter.h>
#include "perception_utils/logger.h"
#include <suturo_perception_cad_recognition/generate_pc_model.h>
#include <boost/algorithm/string.hpp>
// #include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <perception_utils/capability.hpp>

#include <shape_tools/solid_primitive_dims.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <suturo_perception_cad_recognition/IA_centroid.h>

// #include <pcl/registration/ia_ransac.h>
/*
 * This class defines the interface for the perception pipeline.
 * It passes a set of suturo_msgs::Object to the constructor.
 * The Class will then create models from these representations 
 * online and will try to estimate the pose of a given PointCloud
 * against the given models.
 */
class ModelPoseEstimation : public suturo_perception::Capability
{
public:
  // NOT_ENABLED = ModelTransform is disabled
  // TRANSFORM_ALL = ModelTransform is enabled and is applied everytime a pose estimation
  //                 failed
  // TRANSFORM_IF_MOI_SET  = ModelTransform is enabled but will only be considered, if the
  //                         ModelsOfInterest Lists size is > 0
  enum ModelTransformMode { NOT_ENABLED, TRANSFORM_ALL, TRANSFORM_IF_MOI_SET };

  // The objects are the Object specification from the parsed
  // YAML description of the EuRoC Task
  //
  // Note: Pass empty Pointers to ignore the pipelinedata and pipelineobject
  ModelPoseEstimation (boost::shared_ptr<std::vector<suturo_msgs::Object> > objects,suturo_perception::PipelineData::Ptr pipelineData, suturo_perception::PipelineObject::Ptr pipelineObject) : 
  suturo_perception::Capability(pipelineData, pipelineObject)
  {
	  logger_ = suturo_perception::Logger("SuturoPerceptionMPE");

    if(pipelineData == NULL || pipelineObject == NULL)
    {
      logger_.logInfo("pipeline_mode_ = false");
      // std::cout << "pipeline_mode_ = false" << std::endl;
      pipeline_mode_ = false;
      objects_ = objects;
    }
    else
    {
      logger_.logInfo("pipeline_mode_ = true");
      // std::cout << "pipeline_mode_ = true" << std::endl;
      pipeline_mode_ = true;
      // Get the relevant parameters from the pipeline data
      Eigen::Vector4f surface_normal;
      surface_normal[0] = pipelineData->coefficients_->values.at(0);
      surface_normal[1] = pipelineData->coefficients_->values.at(1);
      surface_normal[2] = pipelineData->coefficients_->values.at(2);
      surface_normal[3] = pipelineData->coefficients_->values.at(3);
      surface_normal_ = surface_normal;
      input_cloud_ = pipelineObject->get_pointCloud();

      boost::shared_ptr<std::vector<suturo_msgs::Object> > objects_from_pipeline = boost::make_shared<std::vector<suturo_msgs::Object> >(pipelineData_->task_.objects);

      objects_ = objects_from_pipeline;
    }

    success_threshold_ = 1e-5;
    best_fit_model_ = 0;
    fitness_score_ = 0;
    pose_estimation_successful_ = false;
    generated_models_ = boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> >(new std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> );
    voxel_size_ = 0;
    dump_icp_fitter_pointclouds_ = false;
    remove_nans_ = false;
    max_icp_iterations_ = 60;
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
  // pass a value != 0 . The default is 0.003f. Pass 0 to disable the voxeling.
  void setVoxelSize(double size);

  // Should we call ICPFitter::dumpPointClouds() after execute() has been finished?
  // This writes a pointcloud for every transformation step
  // in suturo_perception_cad_recognition/dumps/
  void setDumpICPFitterPointclouds(bool b);

  // Delete every model and create new ones
  void generateModels();

  // Remove NaNs from the input cloud before the fitting process starts
  // Default: false
  void setRemoveNaNs(bool b);

  // If we are called by the perception pipeline, we can't set the necessary parameters from the calling side
  // We will therefore prepare everything in this method (set voxel size, retrieve models from the task description etc.).
  void initForPipelineCall();

  // This method can alter the current ModelTransformMode. Enabling this mode means,
  // that if not one model matches the desired fitness, the models will 
  // be rotated (by 180 degrees on the Y-Axis, etc.)
  //
  // Please see the documentation of ModelTransformMode for more details on the different modes
  void setTransformModelsOnFail(ModelTransformMode m);



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

  // Get the pose (x,y,z, quaternion.x, quaternion.y, quaternion.z, quaternion.w ) as a 7-dimensional vector.
  // The pose will be the one of the best matching model
  // Available after execute()
  Eigen::VectorXf getEstimatedPose();

  boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> > getGeneratedModels();

  // Returns the name of this class
  std::string getName();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
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
  bool dump_icp_fitter_pointclouds_;
  // true, if the caller has passed pipelineData AND pipelineObjects that are != NULL
  bool pipeline_mode_;
  bool remove_nans_;
  int max_icp_iterations_;

  std::vector<int> parseRequestArgs(std::string req);

};
#endif
