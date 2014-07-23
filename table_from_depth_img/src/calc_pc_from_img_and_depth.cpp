#include "ros/ros.h"
#include <boost/signals2/mutex.hpp>
#include <boost/date_time.hpp>
#include <boost/thread.hpp>
#include <boost/asio.hpp>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud.h>

#include "suturo_perception.h"
#include "perceived_object.h"
#include "point.h"
#include "publisher_helper.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "suturo_perception_utils.h"
#include <sensor_msgs/Image.h>
#include "opencv2/imgproc/imgproc.hpp"

namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Depth Image";

ros::Publisher pub_cloud;

// Thanks for Jan for the code
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
     constant = 1.0f / (0.8203125 * cloud->width),
     bad_point = std::numeric_limits<float>::quiet_NaN();
 register const int
     centerX = (cloud->width >> 1),
     centerY = (cloud->height >> 1);

#pragma omp parallel for
 for (int y = 0; y < depth_image.rows; ++y)
 {
   register pcl::PointXYZRGB *pPt = &cloud->points[y * depth_image.cols];
   register const float *pDepth = depth_image.ptr<float>(y), v = (y - centerY) * constant;
   register const cv::Vec3b *pBGR = rgb_image.ptr<cv::Vec3b>(y);

   for (register int u = -centerX; u < centerX; ++u, ++pPt, ++pDepth, ++pBGR)
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
     pPt->z = depth;
     pPt->x = u * depth * constant;
     pPt->y = v * depth;
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
	std::cout << "Receiving images" << std::endl;
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	// pcl::fromROSMsg(*inputCloud,*cloud_in);
	//
	cv_bridge::CvImagePtr img_ptr;
	img_ptr = cv_bridge::toCvCopy(inputImage, enc::BGR8);

	cv_bridge::CvImagePtr depth_ptr;
	depth_ptr = cv_bridge::toCvCopy(depthImage, enc::TYPE_32FC1);

	cv::Mat resized_img;
	cv::Mat resized_depth;

  cv::resize( depth_ptr->image, resized_depth, cv::Size(), 0.5, 0.5, cv::INTER_AREA ); // TODO: Try INTER_AREA 
  cv::resize( img_ptr->image, resized_img, cv::Size(), 0.5, 0.5, cv::INTER_AREA ); // TODO: Try INTER_AREA 

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out = 
    depth_project(resized_depth, resized_img);


	// cv::imshow(WINDOW, img_ptr->image);
  // cv::waitKey(3);
  
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*cloud_out, pub_message );
  pub_message.header.frame_id = "head_mount_kinect_rgb_optical_frame";
  pub_message.header.stamp = depthImage->header.stamp;
  pub_cloud.publish(pub_message);
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "calc_pc_from_img_and_depth");
	ros::NodeHandle n;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(n, "/head_mount_kinect_rgb/depth/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_sub(n, "/head_mount_kinect/rgb/image_raw", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, image_sub);

  // sync.registerCallback(boost::bind(&SuturoPerceptionROSNode::receive_depth_and_rgb_image,this, _1, _2));
  sync.registerCallback(boost::bind(&receive_depth_and_rgb_image, _1, _2));

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	cv::destroyWindow(WINDOW);

  pub_cloud = n.advertise<sensor_msgs::PointCloud2> ("/suturo/halfsized_cloud", 1);


	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
