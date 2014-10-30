#ifndef PIPELINE_DATA_H
#define PIPELINE_DATA_H

#include <perception_utils/logger.h>
#include <suturo_msgs/Task.h>

#include <pcl/ModelCoefficients.h>
#include <boost/shared_ptr.hpp>

namespace suturo_perception
{
	class PipelineData
	{
    public:
      typedef boost::shared_ptr<PipelineData> Ptr;

      // segmentation results
      pcl::ModelCoefficients::Ptr coefficients_; 
      
      // service parameter
      std::string request_parameters_;
      
      // euroc task description
      suturo_msgs::Task task_;
			
			// call time
			ros::Time stamp;
      
      
      PipelineData()
      {
        logger = Logger("pipeline_data");
        
        // Set default parameters
        zAxisFilterMin = 0.0f;
        zAxisFilterMax = 1.5f;
        downsampleLeafSize = 0.005f; // [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow
        planeMaxIterations = 1000;
        planeDistanceThreshold = 0.001f; 
        ecClusterTolerance = 0.02f; // 2cm
        ecMinClusterSize = 1000;
        ecMaxClusterSize = 200000;  
        prismZMin = 0.003f;
        prismZMax = 0.50f; // cutoff 50 cm above plane
        ecObjClusterTolerance = 0.03f; // 3cm
        ecObjMinClusterSize = 100;
        ecObjMaxClusterSize = 25000;
        numThreads_ = 8;
        mpeMaxICPIterations = 60;
        mpeSuccessThreshold = 1e-5;
        mpeVoxelSize = 0.003;
        mpeDumpICPFitterPointClouds = false;
      }

      // pipeline configuration
      float zAxisFilterMin;
      float zAxisFilterMax;
      float downsampleLeafSize;
      int planeMaxIterations;
      double planeDistanceThreshold;
      double ecClusterTolerance; 
      int ecMinClusterSize;
      int ecMaxClusterSize;  
      double prismZMin;
      double prismZMax;
      double ecObjClusterTolerance;
      int ecObjMinClusterSize;
      int ecObjMaxClusterSize;
      int numThreads_;
      int mpeMaxICPIterations;
      double mpeSuccessThreshold;
      double mpeVoxelSize;
      bool mpeDumpICPFitterPointClouds;
      
      void resetData()
      {
        //coefficients_ = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients);
      }
      
      void printConfig()
      {
        logger.logInfo((boost::format("Reconfigure request : \n"
            "segmenter: zAxisFilterMin: %f \n"
            "segmenter: zAxisFilterMax: %f \n"
            "segmenter: downsampleLeafSize: %f \n"
            "segmenter: planeMaxIterations: %i \n"
            "segmenter: planeDistanceThreshold: %f \n"
            "segmenter: ecClusterTolerance: %f \n"
            "segmenter: ecMinClusterSize: %i \n"
            "segmenter: ecMaxClusterSize: %i \n"
            "segmenter: prismZMin: %f \n"
            "segmenter: prismZMax: %f \n"
            "segmenter: ecObjClusterTolerance: %f \n"
            "segmenter: ecObjMinClusterSize: %i \n"
            "segmenter: ecObjMaxClusterSize: %i \n"
            "general: numThreads: %i \n"
            "mpe: mpeMaxICPIterations: %i \n"
            "mpe: mpeSuccessThreshold: %f \n"
            "mpe: mpeVoxelSize: %f \n"
            "mpe: mpeDumpICPFitterPointClouds: %i \n"
            ) %
            zAxisFilterMin % zAxisFilterMax % downsampleLeafSize %
            planeMaxIterations % planeDistanceThreshold % ecClusterTolerance %
            ecMinClusterSize % ecMaxClusterSize % prismZMin % prismZMax %
            ecObjClusterTolerance % ecObjMinClusterSize % ecObjMaxClusterSize % 
            numThreads_ % mpeMaxICPIterations % mpeSuccessThreshold % mpeVoxelSize % 
            mpeDumpICPFitterPointClouds
            ).str());
      }
    private:
      Logger logger;
	};
}

#endif
