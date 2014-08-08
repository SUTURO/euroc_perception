#include "suturo_scene_node.h"

#include "perception_utils/point_cloud_operations.h"
#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <suturo_perception_segmentation/projection_segmenter.h>

#include <pcl/filters/passthrough.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <shape_msgs/Plane.h>
#include <moveit_msgs/CollisionObject.h>
#include <visualization_msgs/Marker.h>

#include <boost/asio.hpp>

using namespace suturo_perception;

SuturoSceneNode::SuturoSceneNode(ros::NodeHandle &n, std::string imageTopic, std::string depthTopic) : 
  nodeHandle_(n), 
  imageTopic_(imageTopic),
  cloudTopic_(depthTopic)
{
	logger = Logger("SuturoPerceptionSceneNode");
  clusterService_ = nodeHandle_.advertiseService("/suturo/GetScene", 
    &SuturoSceneNode::getScene, this);
	idx_ = 0;

  markerPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/suturo/cuboid_markers", 0);
  maxMarkerId_ = 0;

  }

void SuturoSceneNode::publish_marker(PipelineObject::VecPtr &objects)
{
  logger.logInfo("Publishing centroid marker");

  for (int i = 0; i < maxMarkerId_; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/sdepth"; // TODO: more dynamic?
    marker.header.stamp = ros::Time();
    marker.ns = "suturo_perception";
    marker.id = i;
    marker.action = visualization_msgs::Marker::DELETE;
    markerPublisher_.publish(marker);
  }

  for (int i = 0; i < objects.size(); i++)
  {
    PipelineObject::Ptr obj = objects[i];
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/sdepth"; // TODO: more dynamic?
    marker.header.stamp = ros::Time();
    marker.ns = "suturo_perception";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = obj->get_c_centroid().x;
    marker.pose.position.y = obj->get_c_centroid().y;
    marker.pose.position.z = obj->get_c_centroid().z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    markerPublisher_.publish(marker);
  }

  maxMarkerId_ = objects.size();
}

bool
SuturoSceneNode::getScene(suturo_perception_msgs::GetScene::Request &req, suturo_perception_msgs::GetScene::Response &res)
{
	res.id = idx_;
	idx_++;

  pipelineData_ = PipelineData::Ptr(new PipelineData());
	
	ros::Subscriber sub = nodeHandle_.subscribe<sensor_msgs::PointCloud2>(cloudTopic_, 1, boost::bind(&SuturoSceneNode::receive_cloud,this, _1));
	
	logger.logInfo("Waiting for processed cloud");
  ros::Rate r(20); // 20 hz
  // cancel service call, if no cloud is received after 10s
  boost::posix_time::ptime cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(10);
	processing_ = true;
  while(processing_)
  {
    if(boost::posix_time::second_clock::local_time() >= cancelTime)
    {
      processing_ = false;
			logger.logError("No sensor data available. Aborting.");
			return false;
		}
    ros::spinOnce();
    r.sleep();
  }
  logger.logInfo("done with segmentation, starting pipeline");
  
  /*
  if (coefficients_->values.size() != 4)
	{
		logger.logError("coefficients_.size() != 4");
		return false;
	}
	shape_msgs::Plane plane;
	for (int i = 0; i < 4; i++)
	{
		plane.coef[i] = coefficients_->values.at(i);
	}
  moveit_msgs::CollisionObject table_msg;
	table_msg.planes.push_back(plane);
  //res.table = table_msg;
  */
  
  /****************************************************************************/
  // Execution pipeline
  // Each capability provides an enrichment for the pipelineObject

  // initialize threadpool
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  std::auto_ptr<boost::asio::io_service::work> work(
    new boost::asio::io_service::work(ioService));

  // Add worker threads to threadpool
  for(int i = 0; i < pipelineData_->numThreads_; ++i)
  {
    threadpool.create_thread(
      boost::bind(&boost::asio::io_service::run, &ioService)
      );
  }

  std::vector<CuboidMatcher*> cmvec;
  for (int i = 0; i < pipelineObjects_.size(); i++) 
  {
    // Initialize Capabilities
    
    // suturo_perception_3d_capabilities::CuboidMatcherAnnotator cma(perceivedObjects[i]);
    // Init the cuboid matcher with the table coefficients
    CuboidMatcher *cm = new CuboidMatcher(pipelineObjects_[i]);
		cmvec.push_back(cm);
    cm->setMode(CUBOID_MATCHER_MODE_WITH_COEFFICIENTS);
    cm->setTableCoefficients(pipelineData_->coefficients_);
    cm->setInputCloud(pipelineObjects_[i]->get_pointCloud());

    // post work to threadpool
    ioService.post(boost::bind(&CuboidMatcher::execute, cm, pipelineObjects_[i]->get_c_cuboid()));
  }
  //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  // wait for thread completion.
  // destroy the work object to wait for all queued tasks to finish
  work.reset();
  ioService.run();
  threadpool.join_all();
	
  for (int i = 0; i < cmvec.size(); i++) 
  {
		delete cmvec.at(i);
	}
  /****************************************************************************/

  logger.logInfo("done with perception pipeline, sending result");
  for (int i = 0; i < pipelineObjects_.size(); i++)
  {
    logger.logInfo("sending object");
    if (pipelineObjects_.at(i) == NULL)
    {
      logger.logError("pipeline object is NULL! investigate this!");
      continue;
    }
    res.objects.push_back(pipelineObjects_[i]->toEurocObject());
  }

  logger.logInfo("results sent, publishing markers");
  publish_marker(pipelineObjects_);

  return true;
}


void
SuturoSceneNode::receive_cloud(const sensor_msgs::PointCloud2ConstPtr& inputCloud)
{
	logger.logInfo("image and cloud incoming...");
  if (!processing_)
  {
    logger.logInfo("processing_ == false, aborting...");
    return;
  }
  boost::posix_time::ptime s = boost::posix_time::microsec_clock::local_time();
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::fromROSMsg(*inputCloud,*cloud_in);
  
  
  ProjectionSegmenter projection_segmenter;
  if (!projection_segmenter.segment(cloud_in, pipelineData_, pipelineObjects_))
  {
    return;
  }
 
 // TODO: migrate this into a capability! 
    
  /*
    // Calculate the volume of each cluster
    // Create a convex hull around the cluster and calculate the total volume
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_points_from_hull (new pcl::PointCloud<pcl::PointXYZRGB> ());

    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    hull.setInputCloud(it);
    hull.setDimension(3);
    hull.setComputeAreaVolume(true); // This creates alot of output, but it's necessary for getTotalVolume() ....
    hull.reconstruct (*hull_points);

    // Centroid calulcation
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*hull_points, centroid);  

    logger.logInfo((boost::format("Centroid: %s, %s, %s") % centroid[0] % centroid[1] % centroid[2]).str());

		Point centroid_point;
    centroid_point.x = centroid[0];
    centroid_point.y = centroid[1];
    centroid_point.z = centroid[2];
    pipelineObject->set_c_centroid(centroid_point);
  */
		
  //idx_++;
	processing_ = false;
}