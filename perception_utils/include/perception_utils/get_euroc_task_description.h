#ifndef GET_EUROC_TASK_DESCRIPTION_H
#define GET_EUROC_TASK_DESCRIPTION_H

#include <ros/ros.h>
#include <suturo_msgs/Task.h>
#include <perception_utils/logger.h>

namespace suturo_perception
{
  class EurocTaskClient 
  {
  public:
    EurocTaskClient(ros::NodeHandle &n);
    suturo_msgs::Task getTaskDescription();
    bool requestTaskDescription();
    bool requestTaskDescriptionSingle();
    void receiveTaskDescription(const boost::shared_ptr<const suturo_msgs::Task> task);
    
  protected:
    static const std::string YAML_PARS0R_TOPIC;
    
    Logger logger;
    ros::NodeHandle nodeHandle_;
    suturo_msgs::Task taskDescription_;
    bool waitingForDescription_;
  };
}

#endif
