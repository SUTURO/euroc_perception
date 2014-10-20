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

#include "suturo_pointcloud_publisher/projector.h"

namespace enc = sensor_msgs::image_encodings;
namespace po = boost::program_options;
using namespace boost;
using namespace suturo_perception;

ros::Publisher pub_cloud;

Logger logger("publish_conveyor_pointcloud");


int cloud_point_cnt = 3000;

// Parameterize this node for both euroc cams
std::string frame = "";
std::string output_topic = "";
bool verbose = true;
tf::StampedTransform transform_rgb_depth;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr conveyor_cloud;


void generate_conveyor_cloud(/*suturo_msgs::Task task*/)
{
/* YAML info
  conveyor_belt:
    move_direction_and_length: [ 0, -2, 0 ]
    drop_center_point: [0.3, 1, 0.2]
    drop_deviation: [ 0.01, 0.1, 0.01]
    start_speed: 0.005
    end_speed: 0.5
    n_objects: 10
    object_template: red_cube
*/
	pcl::PointXYZRGB v_dp;
	v_dp.x = 0.3;
	v_dp.y = 1.0;
	v_dp.z = 0.2;
	
	pcl::PointXYZRGB v_mdl;
	v_mdl.x = 0.0;
	v_mdl.y = -2.0;
	v_mdl.z = 0.0;
	
	double w = 0.1 * 2;
	
	conveyor_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	conveyor_cloud->height = 1;
	conveyor_cloud->width = cloud_point_cnt;
	conveyor_cloud->is_dense = false;
	conveyor_cloud->points.resize(cloud_point_cnt);
	
	for (int i = 0; i < cloud_point_cnt; i++)
	{
		conveyor_cloud->points[i].x = v_dp.x + v_mdl.x * ((rand() % 1000) / 1000.0);
		conveyor_cloud->points[i].y = v_dp.y + v_mdl.y * ((rand() % 1000) / 1000.0) + w * ((rand() % 1000) / 1000.0);
		conveyor_cloud->points[i].z = v_dp.z + v_mdl.z * ((rand() % 1000) / 1000.0);
	}
}


void publish_conveyor_cloud(const ros::NodeHandle &nodeHandle)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*conveyor_cloud, pub_message );
  pub_message.header.frame_id = frame;
  pub_message.header.stamp = ros::Time(0);
  pub_cloud.publish(pub_message);
  
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  if (verbose)
	{
		std::stringstream ss;
		ss << "publishing conveyor pointcloud on " << output_topic;
		logger.logTime(s, e, ss.str());
	}
}


int main (int argc, char** argv)
{
  frame = "/odom_combined";
  output_topic = "/suturo/euroc_conveyor_cloud";

	ros::init(argc, argv, "publish_conveyor_pointcloud");
	ros::NodeHandle n;

  pub_cloud = n.advertise<sensor_msgs::PointCloud2> (output_topic, 1);

	generate_conveyor_cloud();

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		publish_conveyor_cloud(n);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
