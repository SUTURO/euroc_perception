#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  int seq = 0;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("sdepth_pcl", "srgb",
                               ros::Time(0), transform);
      ROS_INFO("lookupTransform success!");
      tf::Vector3 ot = transform.getOrigin();
      tf::Quaternion qt = transform.getRotation();
      ROS_INFO("  translation: [ %f , %f , %f ]", ot[0], ot[1], ot[2]);
      ROS_INFO("  rotation:    [ %f , %f , %f , %f ]", qt.x(), qt.y(), qt.z(), qt.w());
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    geometry_msgs::PointStamped p1;
    p1.header.seq = seq++;
    p1.header.stamp = ros::Time(0);
    p1.header.frame_id = "sdepth_pcl";
    p1.point.x = 1.0;
    p1.point.y = 0.25;
    p1.point.z = 0.63;

    geometry_msgs::PointStamped p2;

    try{
      listener.transformPoint("srgb", p1, p2);
      ROS_INFO("transform point success: (%f, %f, %f)", p2.point.x, p2.point.y, p2.point.z);
    } catch (...) {
      ROS_ERROR("transform point failed!");
      ros::Duration(1.0).sleep();
    } 

    rate.sleep();
  }

  return 0;
};
