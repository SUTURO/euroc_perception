#include "ros/ros.h"

#include "perception_utils/logger.h"

#include <boost/signals2/mutex.hpp>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <boost/format.hpp>

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

#include <cmath>
#define PI 3.14159265

namespace enc = sensor_msgs::image_encodings;

perception_utils::Logger logger("publish_scene_cloud");

ros::Publisher pub_cloud;

int cloud_idx = 0;

//   scene_depth_cam:
//     camera:
//       horizontal_fov: 1.047
//       image: {height: 480, width: 640}
//     relative_pose:
//       from: scene_rgb_cam
//       pose: [0.0, -0.04, 0.0, 0, 0, 0]
//     update_rate: 30
//   scene_rgb_cam:
//     camera:
//       horizontal_fov: 1.047
//       image: {height: 480, width: 640}
//     pose: [0.2, 0.02, 0, 0, 0, 0]
//     update_rate: 30
pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_project(const cv::Mat &depth_image_in,
const cv::Mat &rgb_image)
{
	cv::Mat depth_image;
	if (depth_image_in.type() == CV_16U)
		depth_image_in.convertTo(depth_image, CV_32F, 0.001, 0.0);
	else
		depth_image = depth_image_in;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	// TODO cloud->header.stamp = time;
	cloud->height = depth_image.rows;
	cloud->width = depth_image.cols;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);
	register const float
			//constant = 1.0f / (0.8203125 * cloud->width),
			constant = 1.0f / (0.817109 * cloud->width),
			bad_point = std::numeric_limits<float>::quiet_NaN();
	register const int
			centerX = (cloud->width >> 1),
			centerY = (cloud->height >> 1);
	
	
	register const double fov_h = 1.047; // TODO: get this from the yaml description
	register const double fov_v = 2.0 * atan( tan( fov_h / 2.0 ) * ((double)depth_image.rows / (double)depth_image.cols) );
	register const double h = tan(fov_h / 2.0);
	register const double v = tan(fov_v / 2.0);
	
	logger.logInfo((boost::format("fov_h = %s, fov_v = %s, h = %s, v = %s") % fov_h % fov_v % h % v).str());

	register const int offset = 8;
	
	#pragma omp parallel for
	for (int y = 0; y < depth_image.rows; ++y)
	{
		register pcl::PointXYZRGB *pPt = &cloud->points[y * depth_image.cols];
		register const float *pDepth = depth_image.ptr<float>(y);
		//register const float v = (y - centerY) * constant;
		register const cv::Vec3b *pBGR = rgb_image.ptr<cv::Vec3b>(y);
		
		pBGR += offset;

		//for (register int u = -centerX; u < centerX; ++u, ++pPt, ++pDepth, ++pBGR)
		for (register int col = 0; col < cloud->width; col++, ++pPt, ++pDepth)
		{
			pPt->r = pBGR->val[2];
			pPt->g = pBGR->val[1];
			pPt->b = pBGR->val[0];

			register const float depth = *pDepth;
			// Check for invalid measurements
			if (isnan(depth) || depth == 0)
			{
				// not valid
				pPt->x = pPt->y = pPt->z = bad_point;
				continue;
			}
 			//pPt->x = depth;
 			//pPt->z = u * depth * constant;
 			//pPt->y = v * depth;
			
			pPt->x = depth;
 			pPt->y = depth * (h - 2.0*h * (double)((double)col / (double)cloud->width));
 			pPt->z = depth * (v - 2.0*v * (double)((double)y / (double)cloud->height));
			//pPt->y = depth * (h - 2.0*h * (col / depth_image.cols));
			//pPt->z = depth * (v - 2.0*v * (y / depth_image.rows));
			
			if (cloud->width - col >= offset)
			{
				++pBGR;
			}
		}
	}
	return cloud;
}

/*
 * Receive callback for the /camera/depth_registered/points subscription
 */
void receive_depth_and_rgb_image(const sensor_msgs::ImageConstPtr& depthImage,
		const sensor_msgs::ImageConstPtr& inputImage)
{
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	
	cv_bridge::CvImagePtr img_ptr;
	img_ptr = cv_bridge::toCvCopy(inputImage, enc::BGR8);

	cv_bridge::CvImagePtr depth_ptr;
	depth_ptr = cv_bridge::toCvCopy(depthImage, enc::TYPE_32FC1);

	cv::Mat resized_img;
	cv::Mat resized_depth;

  cv::resize( depth_ptr->image, resized_depth, cv::Size(), 0.5, 0.5, cv::INTER_AREA );
  cv::resize( img_ptr->image, resized_img, cv::Size(), 0.5, 0.5, cv::INTER_AREA );

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out = 
    depth_project(resized_depth, resized_img);
  
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*cloud_out, pub_message );
  pub_message.header.frame_id = "sdepth";
  pub_message.header.stamp = depthImage->header.stamp;
  pub_cloud.publish(pub_message);
	
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "generate scene pointcloud");
	
	// write pcd
  pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "euroc_cloud_" << cloud_idx << ".pcd";
	cloud_idx++;
  writer.write(ss.str(), *cloud_out);
  std::cerr << "Saved " << cloud_out->points.size () << " data points to " << ss.str().c_str() << std::endl;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "publish_scene_cloud");
	ros::NodeHandle n;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/euroc_interface_node/cameras/scene_depth_cam", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/euroc_interface_node/cameras/scene_rgb_cam", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, image_sub);

  sync.registerCallback(boost::bind(&receive_depth_and_rgb_image, _1, _2));

  pub_cloud = n.advertise<sensor_msgs::PointCloud2> ("/suturo/euroc_scene_cloud", 1);

	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
