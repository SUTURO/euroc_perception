#include <stdio.h>
#include <suturo_perception_cad_recognition/generate_pc_model.h>
#include <suturo_msgs/Task.h>
#include <perception_utils/get_euroc_task_description.h>
#include <perception_utils/logger.h>
#include <boost/format.hpp>

using namespace suturo_perception;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suturo_perception_puzzle_gen_node");
  ros::NodeHandle nh;
  
  Logger logger("puzzle_pcd_gen");
  
  EurocTaskClient taskClient(nh);
  if (!taskClient.requestTaskDescriptionSingle())
    return -1;
  suturo_msgs::Task taskDesc = taskClient.getTaskDescription();
  
  logger.logInfo((boost::format("Task description contains %s puzzle pieces") % taskDesc.objects.size()).str());
}
