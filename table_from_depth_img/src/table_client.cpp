#include "ros/ros.h"
#include "suturo_perception_msgs/GetTable.h"
#include <cstdlib>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "perception_client");

  ros::NodeHandle n;

  ros::ServiceClient clusterClient = n.serviceClient<suturo_perception_msgs::GetTable>("/suturo/GetTable");
  suturo_perception_msgs::GetTable tableSrv;
  tableSrv.request.s = "get";
  ROS_INFO_STREAM("ServiceClient initialized");
  // run until service gets shut down
  while(true)
  {
    if (clusterClient.call(tableSrv))
    {
      ROS_INFO("Table Service call successful");
      ROS_INFO("something: %ld", (long int)tableSrv.response.something);

      ROS_INFO_STREAM("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
    }
    else
    {
      ROS_ERROR("Failed to call service /suturo/GetTable");
      return 1;
    }
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  return 0;
}
