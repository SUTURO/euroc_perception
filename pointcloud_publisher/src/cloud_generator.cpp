#include "suturo_pointcloud_publisher/cloud_generator.h"
#include "suturo_pointcloud_publisher/projector.h"

#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


using namespace suturo_perception;
namespace enc = sensor_msgs::image_encodings;

CloudGenerator::CloudGenerator(ros::NodeHandle &nodeHandle, bool verbose) : nodeHandle_(nodeHandle), verbose_(verbose)
{
	logger = Logger("CloudGenerator");
  pub_cloud_scene = nodeHandle_.advertise<sensor_msgs::PointCloud2> ("/suturo/euroc_scene_cloud", 1);
  pub_cloud_gripper = nodeHandle_.advertise<sensor_msgs::PointCloud2> ("/suturo/euroc_tcp_cloud", 1);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr CloudGenerator::generateCloud(const bool isTcp, const bool projectColors)
{
	std::string depth_topic, rgb_topic;
	if (isTcp)
	{
    depth_topic = "/euroc_interface_node/cameras/tcp_depth_cam";
    rgb_topic = "/euroc_interface_node/cameras/tcp_rgb_cam";
	}
	else
	{
    depth_topic = "/euroc_interface_node/cameras/scene_depth_cam";
    rgb_topic = "/euroc_interface_node/cameras/scene_rgb_cam";
	}

	if (projectColors)
	{
		message_filters::Subscriber<sensor_msgs::Image> depth_sub(nodeHandle_, depth_topic, 1);
		message_filters::Subscriber<sensor_msgs::Image> image_sub(nodeHandle_, rgb_topic, 1);

		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
		message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, image_sub);

		sync.registerCallback(boost::bind(&CloudGenerator::receive_depth_and_rgb_image, this, _1, _2, isTcp));
	}
	else
	{
		nodeHandle_.subscribe<sensor_msgs::Image>(
    depth_topic, 
    1, 
    boost::bind(&CloudGenerator::receive_depth_image, this, _1, isTcp));
	}

	ros::Rate r(20); // 20 hz
  // cancel service call, if no cloud was received after 5s
  boost::posix_time::ptime cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(5);
  receivedRGBD_ = false;
  while (!receivedRGBD_)
  {
    if(boost::posix_time::second_clock::local_time() >= cancelTime)
    {
      receivedRGBD_ = false;
      logger.logError("No cloud received after 5 seconds. Aborting");
      return pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    ros::spinOnce();
    r.sleep();
  }
  
  return cloud_out_;
}

void CloudGenerator::receive_depth_and_rgb_image(
    const sensor_msgs::ImageConstPtr& depthImage,
		const sensor_msgs::ImageConstPtr& inputImage,
		const bool& isTcp)
{
	if (receivedRGBD_)
	{
		logger.logWarn("Received callback that wasn't requested");
		return;
	}
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  if(verbose_)
    std::cout << "Receiving images" << std::endl;

	cv_bridge::CvImagePtr img_ptr;
	img_ptr = cv_bridge::toCvCopy(inputImage, enc::BGR8);

  if(verbose_)
    std::cout << "Received rgb image" << std::endl;

	cv_bridge::CvImagePtr depth_ptr;
	depth_ptr = cv_bridge::toCvCopy(depthImage, enc::TYPE_32FC1);
  if(verbose_)
    std::cout << "Received depth image" << std::endl;
	

	cv::Mat resized_img;
	cv::Mat resized_depth;
	
	cv::Size expected_size(640,480);
	
	if (img_ptr->image.size() == expected_size)
	{
		resized_img = img_ptr->image.clone();
	}
	else
	{
		std::cout << "WARNING: incoming rgb image doesn't have expected size, resizing from " << img_ptr->image.size().width << "x" << img_ptr->image.size().height << " to " << expected_size.width << "x" << expected_size.height << std::endl;
		cv::resize(img_ptr->image, resized_img, expected_size);
	}
	if (depth_ptr->image.size() == expected_size)
	{
		resized_depth = depth_ptr->image.clone();
	}
	else
	{
		std::cout << "WARNING: incoming depth image doesn't have expected size, resizing from " << depth_ptr->image.size().width << "x" << depth_ptr->image.size().height << " to " << expected_size.width << "x" << expected_size.height << std::endl;
		cv::resize(depth_ptr->image, resized_depth, expected_size);
	}

	std::string frame, frame_rgb;
	if(isTcp)
  {
    frame = "/tdepth_pcl";
    frame_rgb = "/trgb";
  }
  else
  {
    frame = "/sdepth_pcl";
    frame_rgb = "/srgb";
  }
  tf::StampedTransform transform;

	CloudProjector::getTransform(nodeHandle_, frame_rgb, frame, transform );
	if (verbose_)
		CloudProjector::printTransform(transform);
  cloud_out_ = CloudProjector::depthProject(resized_depth, resized_img, transform, true);
	
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*cloud_out_, pub_message );
  pub_message.header.frame_id = frame;
  pub_message.header.stamp = depthImage->header.stamp;
	if (isTcp)
	{
		pub_cloud_gripper.publish(pub_message);
	}
	else
	{
		pub_cloud_scene.publish(pub_message);
	}
  
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  if (verbose_)
	{
		logger.logTime(s, e, "generated colored pointcloud");
	}
	
	receivedRGBD_ = true;
}


void CloudGenerator::receive_depth_image(
    const sensor_msgs::ImageConstPtr& depthImage,
		const bool& isTcp)
{
	if (receivedRGBD_)
	{
		logger.logWarn("Received callback that wasn't requested");
		return;
	}
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  if(verbose_)
    std::cout << "Receiving images" << std::endl;

	cv_bridge::CvImagePtr depth_ptr;
	depth_ptr = cv_bridge::toCvCopy(depthImage, enc::TYPE_32FC1);
  if(verbose_)
    std::cout << "Received depth image" << std::endl;
	
	cv::Mat resized_depth;

  resized_depth = depth_ptr->image.clone();

  cloud_out_ = CloudProjector::depthProject(resized_depth, cv::Mat(), tf::StampedTransform(), false);
	
	std::string frame;
	if(isTcp)
  {
    frame = "/tdepth_pcl";
  }
  else
  {
    frame = "/sdepth_pcl";
  }
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*cloud_out_, pub_message );
  pub_message.header.frame_id = frame;
  pub_message.header.stamp = depthImage->header.stamp;
	if (isTcp)
	{
		pub_cloud_gripper.publish(pub_message);
	}
	else
	{
		pub_cloud_scene.publish(pub_message);
	}
  
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  if (verbose_)
	{
		logger.logTime(s, e, "generated colorless pointcloud");
	}
	
	receivedRGBD_ = true;
}
