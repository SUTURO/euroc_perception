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
#include "pcl_ros/transforms.h"

//
//
boost::mutex mutex_tcp;
boost::mutex mutex_scene;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr latest_scene_cloud,
  latest_tcp_cloud;
tf::TransformListener *tfListener;

void
receive_tcp_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  std::cout << "tcp cloud incoming..." << std::endl;

  sensor_msgs::PointCloud2 transformedCloud;
  pcl_ros::transformPointCloud("/odom_combined", *inputCloud, transformedCloud, *tfListener);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(transformedCloud,*cloud_in);

  mutex_tcp.lock();
  latest_tcp_cloud = cloud_in;
  mutex_tcp.unlock();
}

void
receive_scene_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
  std::cout << "scene cloud incoming..." << std::endl;
  // boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
  sensor_msgs::PointCloud2 transformedCloud;
  pcl_ros::transformPointCloud("/odom_combined", *inputCloud, transformedCloud, *tfListener);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(transformedCloud,*cloud_in);
	
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	// pcl::fromROSMsg(*inputCloud,*cloud_in);
  mutex_scene.lock();
  latest_scene_cloud = cloud_in;
  mutex_scene.unlock();
}

bool execute(suturo_perception_msgs::GetPointArray::Request  &req,
         suturo_perception_msgs::GetPointArray::Response &res)
{
  std::cout << "Service called" << std::endl;

  if(latest_scene_cloud == NULL or latest_tcp_cloud == NULL)
  return false;


  if(req.pointCloudName == suturo_perception_msgs::GetPointArrayRequest::TCP)
  {
     mutex_tcp.lock();
     for (int i = 0; i < latest_tcp_cloud->points.size(); i++) {
       res.pointArray.push_back(latest_tcp_cloud->points.at(i).x);
       res.pointArray.push_back(latest_tcp_cloud->points.at(i).y);
       res.pointArray.push_back(latest_tcp_cloud->points.at(i).z);
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
     }
     mutex_scene.unlock();
  }
  else
  {
    ROS_INFO("Invalid pointCloudName given");
    return false;
  }

  std::cout << "Service call ended" << std::endl;
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
  tfListener = new tf::TransformListener();

  std::cout << "Waiting for tf to come up..." << std::endl;
  sleep(6);

	ros::Subscriber scene_sub = n.subscribe<sensor_msgs::PointCloud2>("/suturo/euroc_scene_cloud", 1, boost::bind(&receive_scene_cloud, _1));

	ros::Subscriber tcp_sub = n.subscribe<sensor_msgs::PointCloud2>("/suturo/euroc_tcp_cloud", 1, boost::bind(&receive_tcp_cloud, _1));

  std::cout << "Susbcribed to topics" << std::endl;
  ros::ServiceServer service = n.advertiseService("/suturo/GetPointArray", execute);
  ros::spin();

  return 0;
}
