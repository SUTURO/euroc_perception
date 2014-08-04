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
