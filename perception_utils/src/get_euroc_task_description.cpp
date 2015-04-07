#include "perception_utils/get_euroc_task_description.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>

using namespace suturo_perception;

const std::string EurocTaskClient::YAML_PARS0R_TOPIC= "/suturo/startup/yaml_pars0r";

EurocTaskClient::EurocTaskClient(ros::NodeHandle &n)
{
  logger = Logger("EurocTaskClient");
  nodeHandle_ = n;
}

bool EurocTaskClient::requestTaskDescription()
{
	int i = 1;
	while (!requestTaskDescriptionSingle())
	{
		logger.logError((boost::format("requesting task description failed! trying again... attempt %s") % i).str());
		i++;
	}
	return true;
}

bool EurocTaskClient::requestTaskDescriptionSingle()
{
  ros::Subscriber sub = nodeHandle_.subscribe<suturo_msgs::Task>(
    YAML_PARS0R_TOPIC, 
    1, 
    boost::bind(&EurocTaskClient::receiveTaskDescription,this, _1));

  ros::Rate r(20); // 20 hz
  // cancel service call, if no task was received after 10s
  boost::posix_time::ptime cancelTime = boost::posix_time::second_clock::local_time() + boost::posix_time::seconds(10);
  waitingForDescription_ = true;
  while (waitingForDescription_)
  {
    if(boost::posix_time::second_clock::local_time() >= cancelTime)
    {
      waitingForDescription_ = false;
      logger.logError("No task received after 10 seconds. Aborting");
      return false;
    }
    ros::spinOnce();
    r.sleep();
  }
  
  return true;
}

suturo_msgs::Task EurocTaskClient::getTaskDescription()
{
  return taskDescription_;
}

void EurocTaskClient::receiveTaskDescription(const boost::shared_ptr<const suturo_msgs::Task> task)
{
  if (!waitingForDescription_)
  {
    logger.logError("received task description that wasn't requested");
    return;
  }
  
  taskDescription_ = *task;
  waitingForDescription_ = false;
}
