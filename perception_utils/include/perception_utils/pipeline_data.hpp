#ifndef PIPELINE_DATA_H
#define PIPELINE_DATA_H

#include <pcl/ModelCoefficients.h>
#include <boost/shared_ptr.hpp>

namespace suturo_perception
{
	class PipelineData
	{
    public:
      typedef boost::shared_ptr<PipelineData> Ptr;
      PipelineData()
      {
        // Set default parameters
        zAxisFilterMin = 0.0f;
        zAxisFilterMax = 1.5f;
        downsampleLeafSize = 0.01f; // [pcl::VoxelGrid::applyFilter] Leaf size is too small for the input dataset. Integer indices would overflow
        planeMaxIterations = 1000;
        planeDistanceThreshold = 0.01f; 
        ecClusterTolerance = 0.02f; // 2cm
        ecMinClusterSize = 1000;
        ecMaxClusterSize = 200000;  
        prismZMin = 0.001f;
        prismZMax = 0.50f; // cutoff 50 cm above plane
        ecObjClusterTolerance = 0.05f; // 3cm
        ecObjMinClusterSize = 10;
        ecObjMaxClusterSize = 25000;
        numThreads_ = 8;
      }

      // segmentation results
      pcl::ModelCoefficients::Ptr coefficients_; 

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
	};
}

#endif
