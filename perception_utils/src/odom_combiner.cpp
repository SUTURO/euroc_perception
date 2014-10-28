/** This node takes a PointCloud from one of the cameras in 
 * the EuRoC Simulation, transforms it to PointCloud2
 * and publishes the points as a vector of float32s.
 *
 * The data array will hold the X,Y,Z values (in that particular
 * order) for every point.
 */

#include "ros/ros.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "suturo_perception_msgs/GetPointArray.h"
#include <tf/transform_listener.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_ros/transforms.h"
#include <perception_utils/node_status.hpp>

//
//
boost::mutex mutex_tcp;
boost::mutex mutex_scene;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_scene_cloud,
  latest_tcp_cloud;
tf::TransformListener *tfListener;
ros::NodeHandle *node_handle;
ros::Publisher pub;

// Safe the timestamps for the latest clouds
ros::Time time_tcp_cloud;
ros::Time time_scene_cloud;

typedef struct HSVColor_ {
  uint32_t h;
  double s;
  double v;
} HSVColor;


HSVColor 
convertRGBToHSV(double r, double g, double b) 
{
  HSVColor hsv;
  // double r = ((uint8_t) ((rgb & 0xff0000) >> 16)) / 255.0;
  // double g = ((uint8_t) ((rgb & 0xff00) >> 8)) / 255.0;
  // double b = ((uint8_t) (rgb & 0xff)) / 255.0;
  //logger.logWarn((boost::format("convertRGBToHSV input: %f, %f, %f") % r % g % b).str());
  double rgb_min, rgb_max;
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(1) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  hsv.v = rgb_max;
  if (hsv.v == 0) {
      hsv.h = hsv.s = 0;
      return hsv;
  }
  /* Normalize value to 1 */
  r /= hsv.v;
  g /= hsv.v;
  b /= hsv.v;
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(2) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  hsv.s = rgb_max - rgb_min;
  if (hsv.s == 0) {
      hsv.h = 0;
      return hsv;
  }
  /* Normalize saturation to 1 */
  r = (r - rgb_min)/(rgb_max - rgb_min);
  g = (g - rgb_min)/(rgb_max - rgb_min);
  b = (b - rgb_min)/(rgb_max - rgb_min);
  rgb_min = std::min(r, std::min(g, b));
  rgb_max = std::max(r, std::max(g, b));
  //logger.logWarn((boost::format("convertRGBToHSV(3) rgb %f, %f, %f, max = %f, min = %f") % r % g % b % rgb_max % rgb_min).str());
  /* Compute hue */
  if (rgb_max == r) {
      int h_tmp = 0.0 + 60.0*(g - b);
      //logger.logWarn((boost::format("convertRGBToHSV hue: %f, %f, %f") % hsv.h % g % b).str());
      if (h_tmp < 0.0) {
          h_tmp += 360.0;
      }
      hsv.h = h_tmp;
      //logger.logWarn((boost::format("convertRGBToHSV hue2: %f") % hsv.h).str());
  } else if (rgb_max == g) {
      hsv.h = 120.0 + 60.0*(b - r);
      //logger.logWarn((boost::format("convertRGBToHSV hue3: %f") % hsv.h).str());
  } else /* rgb_max == b */ {
      hsv.h = 240.0 + 60.0*(r - g);
      //logger.logWarn((boost::format("convertRGBToHSV hue4: %f") % hsv.h).str());
  }
  return hsv;
}

float 
getNearestRGBColor(HSVColor c)
{
  uint32_t &h = c.h;
  double &s   = c.s;
  double &v   = c.v;
  const int hue_tolerance = 20; // 4 is enough for tasks 1,3-6. 6 is required for task 2 // 15 is a better choice if we use the scene cam ...
  const int hue_blue    = 240;
  const int hue_green   = 120;
  const int hue_cyan    = 180;
  const int hue_red     = 0;
  const int hue_magenta = 300;
  const int hue_yellow  = 60;


  // Check saturation first. if it's below 40, not much of the color is left
  if(s < 0.80)
  {
    // logger.logInfo("Saturation too low for color_class");
    return -1;
  }

  if(v < 0.40)
  {
    // logger.logInfo("Value too low for color_class");
    return -1;
  }

  if(h > hue_blue - hue_tolerance && h < hue_blue + hue_tolerance)
  {
    return 255; // blue
  }
  
  if(h > hue_green - hue_tolerance && h < hue_green + hue_tolerance)
  {
    return 65280; // green
  }
  
  if(h > hue_cyan - hue_tolerance && h < hue_cyan + hue_tolerance)
  {
    return 65535; // cyan
  }
  
  // Check for red
  // Check for h in [360-tolerance...360] and [0..hue_tolerance]
  if( (h > (360 - hue_tolerance) && h <= 360) || 
       (h >= hue_red && h < hue_red + hue_tolerance) )
  {
    return 16711680; // red
  }
  
  if(h > hue_magenta - hue_tolerance && h < hue_magenta + hue_tolerance)
  {
    return 16711935; // magenta
  }
  
  if(h > hue_yellow - hue_tolerance && h < hue_yellow + hue_tolerance)
  {
    return 16776960; // yellow
  }

  // No rule matched - return unknown
  // std::stringstream hsv;
  // hsv << "No rule matched for HSV: ";
  // hsv << h << " ";
  // hsv << s << " ";
  // hsv << v << " ";

  // logger.logInfo(hsv.str());
  return -1;
  
}
void
receive_tcp_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  // std::cout << "tcp cloud incoming..." << std::endl;

  sensor_msgs::PointCloud2 transformedCloud;
  pcl_ros::transformPointCloud("/odom_combined", *inputCloud, transformedCloud, *tfListener);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxeled_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(transformedCloud,*cloud_in);

  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (0.003f, 0.003f, 0.003f);
  sor.filter (*voxeled_cloud);

  mutex_tcp.lock();
  latest_tcp_cloud = voxeled_cloud;
  time_tcp_cloud = inputCloud->header.stamp;
  mutex_tcp.unlock();
  // std::cout << "pts after voxeling: " << latest_tcp_cloud->points.size() << std::endl;
}

void
receive_scene_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  // std::cout << "scene cloud incoming..." << std::endl;
  // boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  sensor_msgs::PointCloud2 transformedCloud;
  pcl_ros::transformPointCloud("/odom_combined", *inputCloud, transformedCloud, *tfListener);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxeled_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(transformedCloud,*cloud_in);
	
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (0.003f, 0.003f, 0.003f);
  sor.filter (*voxeled_cloud);

  mutex_scene.lock();
  latest_scene_cloud = voxeled_cloud;
  time_scene_cloud = inputCloud->header.stamp;
  mutex_scene.unlock();
  // std::cout << "pts after voxeling: " << latest_scene_cloud->points.size() << std::endl;
}

bool execute(suturo_perception_msgs::GetPointArray::Request  &req,
         suturo_perception_msgs::GetPointArray::Response &res)
{
  // std::cout << "Service called" << std::endl;

  if(latest_scene_cloud == NULL or latest_tcp_cloud == NULL)
  return false;


  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  if(req.pointCloudName == suturo_perception_msgs::GetPointArrayRequest::TCP)
  {
     mutex_tcp.lock();
     for (int i = 0; i < latest_tcp_cloud->points.size(); i++) {
       res.pointArray.push_back(latest_tcp_cloud->points.at(i).x);
       res.pointArray.push_back(latest_tcp_cloud->points.at(i).y);
       res.pointArray.push_back(latest_tcp_cloud->points.at(i).z);
       HSVColor c = convertRGBToHSV(latest_tcp_cloud->points.at(i).r,
       latest_tcp_cloud->points.at(i).g, latest_tcp_cloud->points.at(i).b);
       res.pointArray.push_back( getNearestRGBColor(c) );
     }
     if(req.publishToPlanningScene)
     {
       sensor_msgs::PointCloud2 pub_message;
       pcl::toROSMsg(*latest_tcp_cloud, pub_message );
       pub_message.header.frame_id = "/odom_combined";
       pub_message.header.stamp = time_tcp_cloud;
       pub.publish(pub_message);
     }
     mutex_tcp.unlock();
  }
  else if(req.pointCloudName == suturo_perception_msgs::GetPointArrayRequest::SCENE)
  {
     // res.pointArray 
     mutex_scene.lock();
     for (int i = 0; i < latest_scene_cloud->points.size(); i++) {
       res.pointArray.push_back(latest_scene_cloud->points.at(i).x);
       res.pointArray.push_back(latest_scene_cloud->points.at(i).y);
       res.pointArray.push_back(latest_scene_cloud->points.at(i).z);
       HSVColor c = convertRGBToHSV(latest_scene_cloud->points.at(i).r,
       latest_scene_cloud->points.at(i).g, latest_scene_cloud->points.at(i).b);
       res.pointArray.push_back( getNearestRGBColor(c) );
     }
     if(req.publishToPlanningScene)
     {
       sensor_msgs::PointCloud2 pub_message;
       pcl::toROSMsg(*latest_scene_cloud, pub_message );
       pub_message.header.frame_id = "/odom_combined";
       pub_message.header.stamp = time_scene_cloud;
       pub.publish(pub_message);
     }
     mutex_scene.unlock();
  }
  else
  {
    ROS_INFO("Invalid pointCloudName given");
    return false;
  }

  boost::posix_time::ptime e = boost::posix_time::microsec_clock::local_time();

  boost::posix_time::time_duration d = e - s;
  float diff = (float)d.total_microseconds() / (float)1000;
  // std::cout << (boost::format("Time for %s: %s ms") % "execute()" % diff).str();

  // std::cout << "Service call ended" << std::endl;
  // res.sum = req.a + req.b;
  // ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  // ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

// TODO throw error if latest_scene_cloud or latest_tcp_cloud is NULL

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_combiner");
  ros::NodeHandle n;
  node_handle = &n;

  pub = node_handle->advertise<sensor_msgs::PointCloud2>("/suturo/octomap", 1000);
  tfListener = new tf::TransformListener();

  std::cout << "[odom_combiner] Waiting for tf to come up..." << std::endl;
  sleep(6);

	ros::Subscriber scene_sub = n.subscribe<sensor_msgs::PointCloud2>("/suturo/euroc_scene_cloud", 1, boost::bind(&receive_scene_cloud, _1));

	ros::Subscriber tcp_sub = n.subscribe<sensor_msgs::PointCloud2>("/suturo/euroc_tcp_cloud", 1, boost::bind(&receive_tcp_cloud, _1));

  std::cout << "[odom_combiner] Subscribed to topics" << std::endl;
  ros::ServiceServer service = n.advertiseService("/suturo/GetPointArray", execute);
	
	suturo_perception::NodeStatus node_status(n);
	node_status.nodeStarted(suturo_perception_msgs::PerceptionNodeStatus::NODE_ODOM_COMBINER);
	
  ros::spin();

  return 0;
}
