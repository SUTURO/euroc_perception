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
#include <perception_utils/node_status.hpp>

#include "suturo_pointcloud_publisher/projector.h"

namespace enc = sensor_msgs::image_encodings;
namespace po = boost::program_options;
using namespace boost;
using namespace suturo_perception;

ros::Publisher pub_cloud;

Logger logger("publish_pointcloud");

int cloud_idx = 0;

// Parameterize this node for both euroc cams
std::string depth_topic = "";
std::string rgb_topic = "";
std::string frame = "";
std::string frame_rgb = "";
std::string output_topic = "";
unsigned char status_node_type;
bool verbose = false;
tf::StampedTransform transform_rgb_depth;



/*
 * Receive callback for the /camera/depth_registered/points subscription
 */
void receive_depth_and_rgb_image(
    const ros::NodeHandle &nodeHandle,
    const sensor_msgs::ImageConstPtr& depthImage,
		const sensor_msgs::ImageConstPtr& inputImage,
		const bool projectColors)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  if(verbose)
    std::cout << "Receiving images" << std::endl;

	cv_bridge::CvImagePtr img_ptr;
	img_ptr = cv_bridge::toCvCopy(inputImage, enc::BGR8);

  if(verbose)
    std::cout << "Received rgb image" << std::endl;

	cv_bridge::CvImagePtr depth_ptr;
	depth_ptr = cv_bridge::toCvCopy(depthImage, enc::TYPE_32FC1);
  if(verbose)
    std::cout << "Received depth image" << std::endl;
	
  // std::cout << "RGB.timestamp - Depth.timestamp = " << inputImage->header.stamp - depthImage->header.stamp << std::endl;
	cv::Mat resized_img;
	cv::Mat resized_depth;

  resized_img = img_ptr->image.clone();
  resized_depth = depth_ptr->image.clone();

  tf::StampedTransform transform;
	if (projectColors)
	{
    transform = transform_rgb_depth;
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out = 
    CloudProjector::depthProject(resized_depth, resized_img, transform, projectColors);

	// write pcd
  // pcl::PCDWriter writer;
    /*
  std::stringstream ss;
  ss << "euroc_cloud_" << cloud_idx << ".pcd";
	cloud_idx++;
  // writer.write(ss.str(), *cloud_out);
  if(verbose)
    std::cerr << "Saved " << cloud_out->points.size () << " data points" << std::endl;
  */
	
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*cloud_out, pub_message );
  pub_message.header.frame_id = frame;
  pub_message.header.stamp = depthImage->header.stamp;
  pub_cloud.publish(pub_message);
  
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  if (verbose)
	{
		std::stringstream ss;
		ss << "generate " << (projectColors?"":"fast ") << "pointcloud on " << output_topic;
		logger.logTime(s, e, ss.str());
	}
}

int main (int argc, char** argv)
{
  depth_topic = "/euroc_interface_node/cameras/scene_depth_cam";
  rgb_topic = "/euroc_interface_node/cameras/scene_rgb_cam";
  frame = "/sdepth_pcl";
  output_topic = "/suturo/euroc_scene_cloud";
  std::string desired_cam = "scene";
	status_node_type = suturo_perception_msgs::PerceptionNodeStatus::NODE_CLOUD_SCENE;
	bool project_colors = true;

  // "HashMap" for program parameters
  po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("cam,c", po::value<std::string>(&desired_cam)->required(), "Specify the camera for which the pointclouds should be generated. Allowed values: scene or tcp")
      ("rgb,s", po::value<bool>(&project_colors), "Dis/Enable colored pointclouds. Allowed values: 0 or 1")
      ("verbose,v", po::value<bool>()->zero_tokens(), "Verbose output")
    ;

    po::positional_options_description p;
    // po::store(po::command_line_parser(argc, argv).
    // options(desc).positional(p).run(), vm); 
    po::store(po::command_line_parser(argc, argv).
    options(desc).allow_unregistered().run(), vm); // Allow unknown parameters 

    if (vm.count("help")) {
      std::cout << "Usage: publish_pointcloud -c camera_name [-v]" << std::endl << std::endl;
      std::cout << desc << "\n";
      return 1;
    }
    if(vm.count("verbose"))
    {
      verbose = true;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

  }
  catch(std::exception& e)
  {
    std::cout << "Usage: Usage: publish_pointcloud -c camera_name [-v]" << std::endl << std::endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

  // Set parameters according to desired cam
  if(desired_cam == "scene")
  {
    depth_topic = "/euroc_interface_node/cameras/scene_depth_cam";
    rgb_topic = "/euroc_interface_node/cameras/scene_rgb_cam";
    frame = "/sdepth_pcl";
    frame_rgb = "/srgb";
		//if (project_colors)
			output_topic = "/suturo/euroc_scene_cloud";
		//else
		//	output_topic = "/suturo/euroc_scene_cloud_fast";
		status_node_type = suturo_perception_msgs::PerceptionNodeStatus::NODE_CLOUD_SCENE;
  }
  else if(desired_cam == "tcp")
  {
    depth_topic = "/euroc_interface_node/cameras/tcp_depth_cam";
    rgb_topic = "/euroc_interface_node/cameras/tcp_rgb_cam";
    frame = "/tdepth_pcl";
    frame_rgb = "/trgb";
		//if (project_colors)
			output_topic = "/suturo/euroc_tcp_cloud";
		//else
		//	output_topic = "/suturo/euroc_gripper_cloud_fast";
		status_node_type = suturo_perception_msgs::PerceptionNodeStatus::NODE_CLOUD_GRIPPER;
  }
  else
  {
    std::cout << "Unknown cam name:" << desired_cam << std::endl;
    std::cout << "Please use 'scene' or 'tcp' as a parameter" << std::endl;
    return 1;
  }

  // Construct node name
  std::stringstream ss;
  ss << "publish_pointcloud" << "_" << desired_cam;
	ros::init(argc, argv, ss.str());
	ros::NodeHandle n;

	if (project_colors)
	{
    logger.logInfo("Waiting for tf to come up");
    sleep(6);
    logger.logInfo("Getting RGB<->Depth transform once...");
		CloudProjector::getTransform(n, frame_rgb, frame, transform_rgb_depth);
		if (verbose)
			CloudProjector::printTransform(transform_rgb_depth);
	}

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, depth_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, rgb_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, image_sub);

  sync.registerCallback(boost::bind(&receive_depth_and_rgb_image, n, _1, _2, project_colors));
  logger.logInfo("Subscribed to image and depth topic");

  pub_cloud = n.advertise<sensor_msgs::PointCloud2> (output_topic, 1);

	NodeStatus node_status(n);
	node_status.nodeStarted(status_node_type);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
