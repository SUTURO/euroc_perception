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

#include "projector.h"

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
bool verbose = false;

void printTransform(const tf::StampedTransform &transform)
{ 
  tf::Vector3 ot = transform.getOrigin();
  tf::Quaternion qt = transform.getRotation();
  ROS_INFO("  translation: [ %f , %f , %f ]", ot[0], ot[1], ot[2]);
  ROS_INFO("  rotation:    [ %f , %f , %f , %f ]", qt.x(), qt.y(), qt.z(), qt.w());
}

bool getTransform(const ros::NodeHandle &node, const ros::Time &t, tf::StampedTransform &transform_) 
{
  tf::TransformListener listener;
  int tries = 0;
  ros::Rate rate(10.0);
  while (node.ok() && tries < 10)
  {
    try
    {
      listener.waitForTransform(frame_rgb, frame, ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(frame_rgb, frame, ros::Time(0), transform_);
      return true;
    }
    catch (tf::TransformException &ex) 
    {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    rate.sleep();
    tries++;
  }
  return false;
}


/*
 * Receive callback for the /camera/depth_registered/points subscription
 */
void receive_depth_and_rgb_image(
    const ros::NodeHandle &nodeHandle,
    const sensor_msgs::ImageConstPtr& depthImage,
		const sensor_msgs::ImageConstPtr& inputImage)
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
	

	cv::Mat resized_img;
	cv::Mat resized_depth;

  resized_img = img_ptr->image.clone();
  resized_depth = depth_ptr->image.clone();

  tf::StampedTransform transform;
  getTransform(nodeHandle, depthImage->header.stamp, transform);
  if (verbose)
    printTransform(transform);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out = 
    CloudProjector::depth_project(resized_depth, resized_img, transform);

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
  // Commented by PM. Generates a huge amount of output that interferes
  // with the pipeline output
  // std::stringstream ss;
  // ss << "generate pointcloud on " << output_topic;
  // logger.logTime(s, e, ss.str());
}

int main (int argc, char** argv)
{
  depth_topic = "/euroc_interface_node/cameras/scene_depth_cam";
  rgb_topic = "/euroc_interface_node/cameras/scene_rgb_cam";
  frame = "/sdepth_pcl";
  output_topic = "/suturo/euroc_scene_cloud";
  std::string desired_cam = "scene";

  // "HashMap" for program parameters
  po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("cam,c", po::value<std::string>(&desired_cam)->required(), "Specify the camera for which the pointclouds should be generated. Allowed values: scene or tcp")
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
    output_topic = "/suturo/euroc_scene_cloud";
  }
  else if(desired_cam == "tcp")
  {
    depth_topic = "/euroc_interface_node/cameras/tcp_depth_cam";
    rgb_topic = "/euroc_interface_node/cameras/tcp_rgb_cam";
    frame = "/tdepth_pcl";
    frame_rgb = "/trgb";
    output_topic = "/suturo/euroc_tcp_cloud";
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

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, depth_topic, 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, rgb_topic, 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, image_sub);

  sync.registerCallback(boost::bind(&receive_depth_and_rgb_image, n, _1, _2));

  pub_cloud = n.advertise<sensor_msgs::PointCloud2> (output_topic, 1);


	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
