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

#include <tf/transform_broadcaster.h>

#include <boost/program_options.hpp>

namespace enc = sensor_msgs::image_encodings;
namespace po = boost::program_options;
using namespace boost;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Depth Image";

ros::Publisher pub_cloud;

int cloud_idx = 0;

// Parameterize this node for both euroc cams
std::string depth_topic = "";
std::string rgb_topic = "";
std::string frame = "";
std::string output_topic = "";
bool verbose = false;

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

#pragma omp parallel for
 for (int y = 0; y < depth_image.rows; ++y)
 {
   pcl::PointXYZRGB *pPt = &cloud->points[y * depth_image.cols];

   const float *pDepth = depth_image.ptr<float>(y);
   const cv::Vec3b *pBGR = rgb_image.ptr<cv::Vec3b>(y);

   for (register int u = 0; u < centerX*2; ++u, ++pPt, ++pDepth, ++pBGR)
   {
     pPt->r = pBGR->val[2];
     pPt->g = pBGR->val[1];
     pPt->b = pBGR->val[0];

     float depth = *pDepth;
     // Check for invalid measurements
     if (isnan(depth) || depth == 0)
     {
       // not valid
       pPt->x = pPt->y = pPt->z = bad_point;
       continue;
     }
     pPt->z = depth;
     pPt->y = depth * (h1 - (2*h1 *( u/(double)640) ));
     pPt->x = depth * (v1 - (2*v1 *( y/(double)480) ));
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

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out = 
    depth_project(resized_depth, resized_img);

	// write pcd
  // pcl::PCDWriter writer;
  std::stringstream ss;
  ss << "euroc_cloud_" << cloud_idx << ".pcd";
	cloud_idx++;
  // writer.write(ss.str(), *cloud_out);
  if(verbose)
    std::cerr << "Saved " << cloud_out->points.size () << " data points" << std::endl;
	
	// cv::imshow(WINDOW, img_ptr->image);
  // cv::waitKey(3);
  
  sensor_msgs::PointCloud2 pub_message;
  pcl::toROSMsg(*cloud_out, pub_message );
  pub_message.header.frame_id = frame;
  pub_message.header.stamp = depthImage->header.stamp;
  pub_cloud.publish(pub_message);
}

int main (int argc, char** argv)
{
  depth_topic = "/euroc_interface_node/cameras/scene_depth_cam";
  rgb_topic = "/euroc_interface_node/cameras/scene_rgb_cam";
  frame = "/sdepth";
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
    frame = "/sdepth";
    output_topic = "/suturo/euroc_scene_cloud";
  }
  else if(desired_cam == "tcp")
  {
    depth_topic = "/euroc_interface_node/cameras/tcp_depth_cam";
    rgb_topic = "/euroc_interface_node/cameras/tcp_rgb_cam";
    frame = "/tdepth";
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

  sync.registerCallback(boost::bind(&receive_depth_and_rgb_image, _1, _2));

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	cv::destroyWindow(WINDOW);

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
