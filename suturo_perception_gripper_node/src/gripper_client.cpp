#include "ros/ros.h"
#include "suturo_perception_msgs/GetGripper.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

int main(int argc, char **argv)
{
  std::cerr << "This suturo perception client is deprecated. Please use rosrun suturo_perception_node suturo_perception_client --type gripper" << std::endl;
  return 1;
  ros::init(argc, argv, "perception_client");

  ros::NodeHandle n;

  ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetGripper>("/suturo/GetGripper");
  suturo_perception_msgs::GetGripper gripperSrv;
  gripperSrv.request.s = "get";
  ROS_INFO_STREAM("ServiceClient initialized");
  // run until service gets shut down
  while(true)
  {
    if (clusterClient.call(gripperSrv))
    {
      ROS_INFO("Scene Service call successful");
			ROS_INFO(" id = %d", gripperSrv.response.id);
			ROS_INFO(" number of detected objects = %d", gripperSrv.response.objects.size());
      for (int i = 0; i < gripperSrv.response.objects.size(); i++)
      {
        suturo_perception_msgs::EurocObject obj = gripperSrv.response.objects.at(i);
        ROS_INFO(" Object %d", i);
        ROS_INFO(" |-> c_id: %d", obj.c_id);
        ROS_INFO(" |-> frame_id: %s", obj.frame_id.c_str());
        ROS_INFO(" |-> centroid: %f %f %f", obj.c_centroid.x, obj.c_centroid.y, obj.c_centroid.z);
        ROS_INFO(" |-> volume: %f", obj.c_volume);
        ROS_INFO(" |-> c_type: %d", obj.c_type);
        ROS_INFO(" |-> c_shape: %d", obj.c_shape);
        ROS_INFO(" |-> c_avg_col_h: %d", obj.c_avg_col_h);
        ROS_INFO(" |-> c_avg_col_s: %f", obj.c_avg_col_s);
        ROS_INFO(" |-> c_avg_col_v: %f", obj.c_avg_col_v);
        ROS_INFO(" |-> c_color_class: %s", obj.c_color_class.c_str() );
        ROS_INFO(" |-> moveit CollisionObject:");
        ROS_INFO(" |-|-> primitives: %d", obj.object.primitives.size());
        for (int j = 0; j < obj.object.primitives.size(); j++)
        {
          ROS_INFO(" |-|-|-> type: %d", obj.object.primitives[j].type);
          if (obj.object.primitives[j].type == shape_msgs::SolidPrimitive::BOX)
          {
            ROS_INFO(" |-|-|-> BOX (x,y,z): (%f,%f,%f)", obj.object.primitives[j].dimensions[shape_msgs::SolidPrimitive::BOX_X], obj.object.primitives[j].dimensions[shape_msgs::SolidPrimitive::BOX_Y], obj.object.primitives[j].dimensions[shape_msgs::SolidPrimitive::BOX_Z]); 
          }
          if (obj.object.primitives[j].type == shape_msgs::SolidPrimitive::CYLINDER)
          {
            ROS_INFO(" |-|-|-> CYLINDER height: %f", obj.object.primitives[j].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]); 
            ROS_INFO(" |-|-|-> CYLINDER radius: %f", obj.object.primitives[j].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]); 
          }
        }
        ROS_INFO(" |-|-> primitive_poses: %d", obj.object.primitive_poses.size());
        for (int k = 0; k < obj.object.primitive_poses.size(); k++)
        {
          ROS_INFO(" |-|-|-> position (x,y,z): (%f,%f,%f)", obj.object.primitive_poses[k].position.x, obj.object.primitive_poses[k].position.y, obj.object.primitive_poses[k].position.z );
          ROS_INFO(" |-|-|-> orientation (x,y,z,w): (%f,%f,%f,%f)", obj.object.primitive_poses[k].orientation.x, obj.object.primitive_poses[k].orientation.y, obj.object.primitive_poses[k].orientation.z, obj.object.primitive_poses[k].orientation.w );
        }


        ROS_INFO(" |-> ModelPose Success: %x", obj.mpe_success);
        ROS_INFO(" |-> ModelPose Object:");
        ROS_INFO(" |-|-> primitives: %d", obj.mpe_object.primitives.size());
        for (int j = 0; j < obj.mpe_object.primitives.size(); j++)
        {
          ROS_INFO(" |-|-|-> type: %d", obj.mpe_object.primitives[j].type);
          if (obj.mpe_object.primitives[j].type == shape_msgs::SolidPrimitive::BOX)
          {
            ROS_INFO(" |-|-|-> BOX (x,y,z): (%f,%f,%f)", obj.mpe_object.primitives[j].dimensions[shape_msgs::SolidPrimitive::BOX_X], obj.mpe_object.primitives[j].dimensions[shape_msgs::SolidPrimitive::BOX_Y], obj.mpe_object.primitives[j].dimensions[shape_msgs::SolidPrimitive::BOX_Z]); 
          }
          if (obj.mpe_object.primitives[j].type == shape_msgs::SolidPrimitive::CYLINDER)
          {
            ROS_INFO(" |-|-|-> CYLINDER height: %f", obj.mpe_object.primitives[j].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]); 
            ROS_INFO(" |-|-|-> CYLINDER radius: %f", obj.mpe_object.primitives[j].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]); 
          }
        }
        ROS_INFO(" |-|-> primitive_poses: %d", obj.mpe_object.primitive_poses.size());
        for (int k = 0; k < obj.mpe_object.primitive_poses.size(); k++)
        {
          ROS_INFO(" |-|-|-> position (x,y,z): (%f,%f,%f)", obj.mpe_object.primitive_poses[k].position.x, obj.mpe_object.primitive_poses[k].position.y, obj.mpe_object.primitive_poses[k].position.z );
          ROS_INFO(" |-|-|-> orientation (x,y,z,w): (%f,%f,%f,%f)", obj.mpe_object.primitive_poses[k].orientation.x, obj.mpe_object.primitive_poses[k].orientation.y, obj.mpe_object.primitive_poses[k].orientation.z, obj.mpe_object.primitive_poses[k].orientation.w );
        }

        ROS_INFO("~~~");
      }
      ROS_INFO_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    }
    else
    {
      ROS_ERROR("Failed to call service /suturo/GetGripper");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}
