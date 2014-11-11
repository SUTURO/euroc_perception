#ifndef CLOUD_GENERATOR_H
#define CLOUD_GENERATOR_H

#include "ros/ros.h"

#include <perception_utils/logger.h>

#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <suturo_msgs/Task.h>

namespace suturo_perception {
  class CloudGenerator
  {
  public:
		CloudGenerator(ros::NodeHandle &nodeHandle, bool verbose);
		
		/**
		 * Generates a single pointcloud from rgbd cams.
		 * @param isTcp generate cloud for gripper (true) or scene (false)
		 * @param projectColors true = cloud will be colored (much slower), false = black points in cloud
		 * @return generated pointcloud
		 */
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr generateCloud(const bool isTcp, const bool projectColors);
		
		// callback for depth and rgb image
		void receive_depth_and_rgb_image(const sensor_msgs::ImageConstPtr& depthImage, const sensor_msgs::ImageConstPtr& inputImage, const bool& isTcp);
		// callback for depth image (no color projection)
		void receive_depth_image(const sensor_msgs::ImageConstPtr& depthImageconst, const bool& isTcp);
	protected:
		bool verbose_;
		bool receivedRGBD_;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_;
		ros::NodeHandle nodeHandle_;
		ros::Publisher pub_cloud_scene;
		ros::Publisher pub_cloud_gripper;
		Logger logger;
  };
}

#endif 