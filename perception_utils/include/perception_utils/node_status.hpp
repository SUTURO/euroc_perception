#ifndef PERCEPTION_NODE_STATUS
#define PERCEPTION_NODE_STATUS

#include <ros/ros.h>
#include <suturo_msgs/Task.h>
#include <perception_utils/logger.h>
#include <perception_utils/get_euroc_task_description.h>
#include <suturo_perception_msgs/PerceptionNodeStatus.h>
#include <boost/format.hpp>
#include <typeinfo>

namespace suturo_perception
{
	class NodeStatus
	{
	public:
		NodeStatus(ros::NodeHandle &node_handle) : node_handle_(node_handle) {
			logger = Logger("NodeStatus");
		}
		
		bool nodeStarted(int node)
		{
			logger.logInfo((boost::format("trying to inform that node %s started") % node).str());
			node_status_publisher = node_handle_.advertise<suturo_perception_msgs::PerceptionNodeStatus> ("/suturo/perception_node_status", 1, true);
			suturo_perception_msgs::PerceptionNodeStatus status_msg;
			status_msg.started_node = node;
			node_status_publisher.publish(status_msg);
			return true;
		}
		
		bool publishRequiredNodes(int task)
		{
			node_status_publisher = node_handle_.advertise<suturo_perception_msgs::PerceptionNodeStatus> ("/suturo/perception_node_status", 1, true);
			suturo_perception_msgs::PerceptionNodeStatus status_msg;
			status_msg.started_node = 200;
			status_msg.required_nodes = getRequiredNodesForTask(task);
			node_status_publisher.publish(status_msg);
			return true;
		}
		
		std::vector<unsigned short> getRequiredNodesForTask(int task)
		{
			std::vector<unsigned short> required_nodes;
			
			switch (task)
			{
				case suturo_msgs::Task::TASK_4:
				case suturo_msgs::Task::TASK_1:
				case suturo_msgs::Task::TASK_2:
				case suturo_msgs::Task::TASK_3:
				case suturo_msgs::Task::TASK_5:
					required_nodes.push_back(suturo_perception_msgs::PerceptionNodeStatus::NODE_ODOM_COMBINER);
					required_nodes.push_back(suturo_perception_msgs::PerceptionNodeStatus::NODE_CLOUD_SCENE);
					required_nodes.push_back(suturo_perception_msgs::PerceptionNodeStatus::NODE_CLOUD_GRIPPER);
					required_nodes.push_back(suturo_perception_msgs::PerceptionNodeStatus::NODE_SCENE);
					required_nodes.push_back(suturo_perception_msgs::PerceptionNodeStatus::NODE_GRIPPER);
					required_nodes.push_back(suturo_perception_msgs::PerceptionNodeStatus::NODE_COLOR_RECOGNIZER);
				break;
				case suturo_msgs::Task::TASK_6:
					required_nodes.push_back(suturo_perception_msgs::PerceptionNodeStatus::NODE_CLOUD_GRIPPER);
					required_nodes.push_back(suturo_perception_msgs::PerceptionNodeStatus::NODE_GRIPPER);
				break;
				default:
					logger.logError((boost::format("Couldn't define which nodes are required! Unknown task_type: %s") % task).str());
				break;
			}
			
			return required_nodes;
		}
		
		
	protected:
		ros::NodeHandle node_handle_;
		ros::Publisher node_status_publisher;
		Logger logger;
	};
}

#endif