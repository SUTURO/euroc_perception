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
#include <opencv/highgui.h>
#include "opencv2/imgproc/imgproc.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <tf/transform_listener.h>

#include <boost/program_options.hpp>

#include <perception_utils/logger.h>

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
std::string frame_depth = "";
std::string frame_rgb = "";
std::string output_topic = "";
bool verbose = false;

// Thanks for Jan for the code
pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_project(const cv::Mat &depth_image_in,
const cv::Mat &rgb_image, const tf::StampedTransform transform)
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
  // cloud->is_dense = false;
  cloud->is_dense = true; // The PC should be dense, since you use the image coords to index the points
  cloud->points.resize(cloud->height * cloud->width);
  register const float
     constant = 1.0f / (0.8203125 * cloud->width),
     bad_point = std::numeric_limits<float>::quiet_NaN();
  register const int
     centerX = (cloud->width >> 1),
     centerY = (cloud->height >> 1);

  // DO THE M(A|E)TH
  double fov_v = 0.817109355f;
  double fov_h = 1.047f;
  double h1 = tan(fov_h/2);
  double v1 = tan(fov_v/2);

  int seq = 0;
  tf::TransformListener listener;

#pragma omp parallel for
  for (int y = 0; y < depth_image.rows; ++y)
  {
   pcl::PointXYZRGB *pPt = &cloud->points[y * depth_image.cols];

   const float *pDepth = depth_image.ptr<float>(y);
   const cv::Vec3b *pBGR = rgb_image.ptr<cv::Vec3b>(y);

   for (register int u = 0; u < centerX*2; ++u, ++pPt, ++pDepth, ++pBGR)
   {
     /*
     pPt->r = pBGR->val[2];
     pPt->g = pBGR->val[1];
     pPt->b = pBGR->val[0];
     */

     float depth = *pDepth;
     // Check for invalid measurements
     if (isnan(depth) || depth == 0)
     {
       // not valid
       pPt->x = pPt->y = pPt->z = bad_point;
       continue;
     }
     /*
     pPt->x = depth;
     pPt->y = depth * (h1 - (2*h1 *( u/(double)640) ));
     pPt->z = depth * (v1 - (2*v1 *( y/(double)480) ));
     */
     
     pPt->z = depth;
     pPt->y = -depth * (v1 - (2*v1 *( y/(double)480) ));
     pPt->x = -depth * (h1 - (2*h1 *( u/(double)640) ));


     geometry_msgs::PointStamped p1;
     p1.header.seq = seq++;
     p1.header.stamp = ros::Time(0);
     p1.header.frame_id = frame;
     p1.point.x = pPt->x;
     p1.point.y = pPt->y;
     p1.point.z = pPt->z;
     
     //tf::Vector3 p3;
     //p3.m_floats[0] = pPt->x;
     //p3.m_floats[1] = pPt->y;
     //p3.m_floats[2] = pPt->z;
     
     //tf::Vector3 p4;

     geometry_msgs::PointStamped p2;

     int tries = 0;
     bool done = false;
     while (tries < 5 && !done)
     {
      try{
        listener.waitForTransform(frame, frame_rgb, ros::Time(0), ros::Duration(3.0));
        listener.transformPoint(frame_rgb, p1, p2);
        //p4 = transform * p3;
        //ROS_INFO("transform point success: (%f, %f, %f)", p2.point.x, p2.point.y, p2.point.z);
        done = true;
      } catch (...) {
      } 
      tries++;
     }
     if (!done)
     {
      //ROS_ERROR("transform point failed!");
      //ros::Duration(1.0).sleep();
      pPt->r = 255;
      pPt->g = 0;
      pPt->b = 255;
      continue;
     }

     int pixx = (( 640.0 * ( p2.point.x * h1 - p2.point.y ) ) / ( 2.0 * p2.point.x * h1 ));
     int pixy = ( 480.0 * ( p2.point.x * v1 - p2.point.z ) ) / ( 2.0 * p2.point.x * v1 );
     //int pixx = (( 640.0 * ( p4.m_floats[0] * h1 - p4.m_floats[1] ) ) / ( 2.0 * p4.m_floats[0] * h1 ));
     //int pixy = ( 480.0 * ( p4.m_floats[0] * v1 - p4.m_floats[2] ) ) / ( 2.0 * p4.m_floats[0] * v1 );
     if (pixx < 0 || pixx > 640 || pixy < 0 || pixy > 480)
     {
      pPt->r = 255;
      pPt->g = 0;
      pPt->b = 255;
     }
     else
     {
      cv::Vec3b bgrPixel = rgb_image.at<cv::Vec3b>(pixy, pixx);
      pPt->r = bgrPixel.val[2];
      pPt->g = bgrPixel.val[1];
      pPt->b = bgrPixel.val[0];
     }
   }
  }
  return cloud;
}

bool getTransform(const ros::NodeHandle &node, const ros::Time &t, tf::StampedTransform *transform_) 
{
  tf::TransformListener listener;

  int tries = 0;
  ros::Rate rate(10.0);
  while (node.ok() && tries < 10){
    tf::StampedTransform transform;
    try{
      listener.waitForTransform(frame, frame_rgb, ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform(frame, frame_rgb,
                               ros::Time(0), transform);
      ROS_INFO("lookupTransform success!");
      *transform_ = transform;
      return true;
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    rate.sleep();
    tries++;
  }

  return false;

}

void printTransform(tf::StampedTransform &transform)
{ 
  tf::Vector3 ot = transform.getOrigin();
  tf::Quaternion qt = transform.getRotation();
  ROS_INFO("  translation: [ %f , %f , %f ]", ot[0], ot[1], ot[2]);
  ROS_INFO("  rotation:    [ %f , %f , %f , %f ]", qt.x(), qt.y(), qt.z(), qt.w());
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
  getTransform(nodeHandle, depthImage->header.stamp, &transform);
  printTransform(transform);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out = 
    depth_project(resized_depth, resized_img, transform);

	// write pcd
  // pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "euroc_cloud_" << cloud_idx << ".pcd";
	cloud_idx++;
  // writer.write(ss.str(), *cloud_out);
  if(verbose)
    std::cerr << "Saved " << cloud_out->points.size () << " data points" << std::endl;
	
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*cloud_out, pub_message );
  pub_message.header.frame_id = frame;
  pub_message.header.stamp = depthImage->header.stamp;
  pub_cloud.publish(pub_message);
  
  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();
  logger.logTime(s, e, "generate pointcloud");
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
