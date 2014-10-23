#include "ros/ros.h"
#include "suturo_perception_msgs/GetGripper.h"
#include "suturo_perception_msgs/GetScene.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 
#include <boost/program_options.hpp>

namespace po = boost::program_options;

void print_euroc_object(suturo_perception_msgs::EurocObject &obj)
{
  ROS_INFO(" |-> c_id: %d", obj.c_id);
  ROS_INFO(" |-> frame_id: %s", obj.frame_id.c_str());
  ROS_INFO(" |-> centroid: %f %f %f", obj.c_centroid.x, obj.c_centroid.y, obj.c_centroid.z);
  ROS_INFO(" |-> volume: %f", obj.c_volume);
  ROS_INFO(" |-> c_type: %d", obj.c_type);
  ROS_INFO(" |-> c_shape: %d", obj.c_shape);
  ROS_INFO(" |-> c_avg_col_h: %d", obj.c_avg_col_h);
  ROS_INFO(" |-> c_avg_col_s: %f", obj.c_avg_col_s);
  ROS_INFO(" |-> c_avg_col_v: %f", obj.c_avg_col_v);
  ROS_INFO(" |-> c_height: %f", obj.c_height);
  ROS_INFO(" |-> c_cuboid_success: %d", obj.c_cuboid_success);
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
}

int main(int argc, char **argv)
{
  // args
  std::string node_type;
  std::string request_s;

  po::variables_map vm;
  po::options_description desc("Allowed options");
  try
  {
    // Declare the supported options.
    desc.add_options()
      ("help", "print help message")
      ("type,t", po::value<std::string>(&node_type)->required(), "Type of the perception client. Allowed values: 'gripper' and 'scene'")
      ("req,r", po::value<std::string>(&request_s)->default_value("get"), "Request string in s")
    ;

    po::positional_options_description p;
    po::store(po::command_line_parser(argc, argv).
    options(desc).allow_unregistered().run(), vm); // Allow unknown parameters 

    if (vm.count("help")) {
      std::cout << "Usage: suturo_perception_client -t NODE_TYPE" << std::endl;
      std::cout << desc << std::endl << std::endl;
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);
  }
  catch (std::exception &e)
  {
    std::cout << "Usage: suturo_perception_client -t NODE_TYPE" << std::endl;
    std::cout << desc << std::endl;
    std::cerr << "Error: " << e.what() << std::endl << std::endl;
    return 1;
  }
  catch(...)
  {
    std::cout << "Usage: suturo_perception_client -t NODE_TYPE" << std::endl;
    std::cout << desc << std::endl << std::endl;
    std::cerr << "Unknown error!" << std::endl;
    return 1;
  } 

  if (node_type == "gripper")
  {
    ros::init(argc, argv, "perception_gripper_client");
    ros::NodeHandle n;
    ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetGripper>("/suturo/GetGripper");
    suturo_perception_msgs::GetGripper gripperSrv;
    gripperSrv.request.s = request_s;
    ROS_INFO_STREAM("GripperServiceClient initialized");
    // run until service gets shut down
    while(true)
    {
      ROS_INFO("Request string: %s", request_s.c_str());
      if (clusterClient.call(gripperSrv))
      {
        ROS_INFO("GripperScene Service call successful");
				ROS_INFO(" stamp = %f", gripperSrv.response.stamp.toSec());
        ROS_INFO(" id = %d", gripperSrv.response.id);
        ROS_INFO(" number of detected objects = %d", gripperSrv.response.objects.size());
        for (int i = 0; i < gripperSrv.response.objects.size(); i++)
        {
          suturo_perception_msgs::EurocObject obj = gripperSrv.response.objects.at(i);
          ROS_INFO(" Object %d", i);
          print_euroc_object(obj);
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
  }
  else
  if (node_type == "scene")
  {
    ros::init(argc, argv, "perception_scene_client");
    ros::NodeHandle n;
    ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetScene>("/suturo/GetScene");
    suturo_perception_msgs::GetScene srv;
    srv.request.s = request_s;
    ROS_INFO_STREAM("SceneServiceClient initialized");
    // run until service gets shut down
    while(true)
    {
      ROS_INFO("Request string: %s", request_s.c_str());
      if (clusterClient.call(srv))
      {
        ROS_INFO("Scene Service call successful");
        ROS_INFO(" id = %d", srv.response.id);
        ROS_INFO(" number of detected objects = %d", srv.response.objects.size());
        for (int i = 0; i < srv.response.objects.size(); i++)
        {
          suturo_perception_msgs::EurocObject obj = srv.response.objects.at(i);
          ROS_INFO(" Object %d", i);
          print_euroc_object(obj);
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
  }
  else
  {
    std::cout << "Usage: suturo_perception_client -t NODE_TYPE" << std::endl;
    std::cout << desc << std::endl << std::endl;
    std::cerr << "Invalid node type specified" << std::endl;
    return 1;
  }

  return 0;
}
