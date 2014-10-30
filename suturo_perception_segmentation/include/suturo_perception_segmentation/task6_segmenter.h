#ifndef SUTURO_PERCEPTION_TASK6_SEGMENTER
#define SUTURO_PERCEPTION_TASK6_SEGMENTER

#include "suturo_perception_segmentation/segmenter.h"
#include <perception_utils/pipeline_data.hpp>
#include <perception_utils/pipeline_object.hpp>
#include <perception_utils/logger.h>
#include <pcl/point_types.h>

namespace suturo_perception
{
  class Task6Segmenter : public Segmenter
  {
    public:
      Task6Segmenter(ros::NodeHandle &node, bool isTcp, suturo_msgs::Task task);

      bool segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, PipelineData::Ptr &pipeline_data, PipelineObject::VecPtr &pipeline_objects);
      // Should be available after a successful segmentation
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr getTablePointCloud();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr getDownsampledPointCloud();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPointsAboveTable();
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr getProjectedPoints();
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getProjectionClusters();
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getProjectionClusterHulls();
			void updateConveyorCloud(PipelineData::Ptr pipeline_data);

    protected:
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_simple_conveyor_cloud();
			
			bool transform_success_;
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr conveyor_cloud_;
			pcl::ModelCoefficients::Ptr table_coefficients_;
			bool isTcp_;
			ros::NodeHandle nodeHandle_;
			suturo_msgs::Task task_;
    private:
      Logger logger;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_pointcloud_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_pointcloud_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_above_table_;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr projected_points_;
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projection_clusters_;
      std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> projection_cluster_hulls_;
  };
}
#endif // SUTURO_PERCEPTION_PROJECTION_SEGMENTER
