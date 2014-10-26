#ifndef IA_METHOD_H
#define IA_METHOD_H

// #include "perception_utils/pipeline_data.hpp"
// #include "perception_utils/pipeline_object.hpp"
// 
// #include <boost/signals2/mutex.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

namespace suturo_perception
{
	class IAMethod
	{
		public:
      IAMethod(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud, Eigen::Vector4f table_normal) : _cloud_in(cloud_in), _model_cloud(model_cloud), _table_normal(table_normal)
      {
      }
      
			boost::posix_time::time_duration getExecutionTime()
			{
				return executeEnd_ - executeStart_;
			}

			virtual void execute() = 0;


      // Get list of rotation matrices, that have been applied during the initial alignment
      std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > getRotations(){ return rotations_; }
      // Get list of translation matrices, that have been applied during the initial alignment
    std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > >  getTranslations(){ return translations_; } 
      // Get a pointcloud for every alignment step
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getObjectTransformationSteps(){ return _object_transformation_steps; }
      // Get the resultant pointcloud after the IA process
      pcl::PointCloud<pcl::PointXYZ>::Ptr getResult(){ return _result; }
      
      // FIX eigen stuff
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			
		protected:
      pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_in;
      pcl::PointCloud<pcl::PointXYZ>::Ptr _model_cloud;
      pcl::PointCloud<pcl::PointXYZ>::Ptr _result;
      Eigen::Vector4f _table_normal;

    std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > rotations_;
    std::vector<Eigen::Matrix< float, 4, 4 >, Eigen::aligned_allocator<Eigen::Matrix< float, 4, 4> > > translations_;
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> _object_transformation_steps;
			
			// time logging
			boost::posix_time::ptime executeStart_;
			boost::posix_time::ptime executeEnd_;
	};
}

#endif
