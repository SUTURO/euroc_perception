#ifndef MODEL_POSE_ESTIMATION_H
#define MODEL_POSE_ESTIMATION_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <suturo_msgs/Task.h>

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
  ModelPoseEstimation (boost::shared_ptr<std::vector<suturo_msgs::Object> > objects);

  //  This should be the observed pointcloud
  void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  // If you want to do the pose estimation only for some of the models
  // (to reduce the search space), you can specify the ids here
  void setModelsOfInterest(std::vector<int> ids);

  // The ICP process will return a fitness score that indicates how well
  // a given model fits to another cloud. You can threshold the return value
  // of ModelPoseEstimation::getPoseEstimationSuccessful with this value
  void setSuccessThreshold(double e);

  void execute();

  // The following methods are useful after execute() has been called
  

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
};
#endif
