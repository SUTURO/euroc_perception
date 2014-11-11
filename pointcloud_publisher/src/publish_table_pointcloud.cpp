#include "ros/ros.h"

#include <boost/signals2/mutex.hpp>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>

#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include "opencv2/imgproc/imgproc.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#include <boost/program_options.hpp>

#include <perception_utils/logger.h>
#include <perception_utils/get_euroc_task_description.h>
#include <suturo_msgs/Task.h>

#include <cmath>

#include "suturo_pointcloud_publisher/projector.h"

#define PI 3.14159265

namespace enc = sensor_msgs::image_encodings;
namespace po = boost::program_options;
using namespace boost;
using namespace suturo_perception;

ros::Publisher pub_cloud;

Logger logger("publish_table_pointcloud");


int cloud_point_cnt = 6000;

// Parameterize this node for both euroc cams
std::string frame = "";
std::string output_topic = "";
bool verbose = true;
tf::StampedTransform transform_rgb_depth;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr table_cloud;

void generate_simple_table_cloud(suturo_msgs::Task task)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	
	table_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	table_cloud->height = 1;
	table_cloud->width = 4;
	table_cloud->is_dense = false;
	table_cloud->points.resize(4);

	table_cloud->points[0].x = -1.0;
	table_cloud->points[0].y = -1.0;
	table_cloud->points[0].z = 0.0;
	table_cloud->points[0].r = 0;
	table_cloud->points[0].g = 255;
	table_cloud->points[0].b = 0;
	
	table_cloud->points[1].x = 1.0;
	table_cloud->points[1].y = -1.0;
	table_cloud->points[1].z = 0.0;
	table_cloud->points[1].r = 255;
	table_cloud->points[1].g = 0;
	table_cloud->points[1].b = 0;
	
	table_cloud->points[2].x = -1.0;
	table_cloud->points[2].y = 1.0;
	table_cloud->points[2].z = 0.0;
	table_cloud->points[2].r = 0;
	table_cloud->points[2].g = 0;
	table_cloud->points[2].b = 255;
	
	table_cloud->points[3].x = 1.0;
	table_cloud->points[3].y = 1.0;
	table_cloud->points[3].z = 0.0;
	table_cloud->points[3].r = 0;
	table_cloud->points[3].g = 255;
	table_cloud->points[3].b = 255;

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  if (verbose)
	{
		std::stringstream ss;
		ss << "generated simple table pointcloud for " << output_topic;
		logger.logTime(s, e, ss.str());
	}
}

void publish_table_cloud(const ros::NodeHandle &nodeHandle)
{
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*table_cloud, pub_message );
  pub_message.header.frame_id = frame;
  pub_message.header.stamp = ros::Time(0);
  pub_cloud.publish(pub_message);
}


int main (int argc, char** argv)
{
  frame = "/odom_combined";
  output_topic = "/suturo/euroc_table_cloud";

	ros::init(argc, argv, "publish_table_pointcloud");
	ros::NodeHandle n;

  pub_cloud = n.advertise<sensor_msgs::PointCloud2> (output_topic, 1);
	
	EurocTaskClient *task_client_ = new EurocTaskClient(n);
  logger.logInfo("Requesting task description");
  if (!task_client_->requestTaskDescription())
  {
    logger.logError("Requesting task description failed. Aborting.");
    return -1;
  }

	generate_simple_table_cloud(task_client_->getTaskDescription());

	if (verbose)
	{
		pcl::PCDWriter writer;
		writer.write("/tmp/table_cloud_.pcd", *table_cloud);
	}

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		publish_table_cloud(n);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
