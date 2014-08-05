#include "ros/ros.h"
#include "suturo_perception_msgs/GetScene.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_client");

  ros::NodeHandle n;

  ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetScene>("/suturo/GetScene");
  suturo_perception_msgs::GetScene sceneSrv;
  sceneSrv.request.s = "get";
  ROS_INFO_STREAM("ServiceClient initialized");
  // run until service gets shut down
  while(true)
  {
    if (clusterClient.call(sceneSrv))
    {
      ROS_INFO("Scene Service call successful");
			ROS_INFO(" id = %d", sceneSrv.response.id);
			ROS_INFO(" number of detected objects = %d", sceneSrv.response.objects.size());
      for (int i = 0; i < sceneSrv.response.objects.size(); i++)
      {
        suturo_perception_msgs::EurocObject obj = sceneSrv.response.objects.at(i);
        ROS_INFO(" Object %d", i);
        ROS_INFO(" |-> c_id: %d", obj.c_id);
        ROS_INFO(" |-> frame_id: %s", obj.frame_id.c_str());
        ROS_INFO(" |-> centroid: %f %f %f", obj.c_centroid.x, obj.c_centroid.y, obj.c_centroid.z);
        ROS_INFO(" |-> volume: %f", obj.c_volume);
        ROS_INFO(" |-> c_type: %d", obj.c_type);
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

        ROS_INFO("~~~");
      }
      ROS_INFO_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    }
    else
    {
      ROS_ERROR("Failed to call service /suturo/GetScene");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}
