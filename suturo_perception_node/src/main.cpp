#include "ros/ros.h"
#include <ros/package.h>
#include "suturo_perception_node.h"
#include <boost/program_options.hpp>

namespace po = boost::program_options;

int main (int argc, char** argv)
{
  // args
  std::string node_type;

  // result of arg parsing
	std::string imageTopic;
  std::string cloudTopic;
  SuturoPerceptionNode::NodeType node_type_enum;

  po::variables_map vm;
  po::options_description desc("Allowed options");
  try
  {
    // Declare the supported options.
    desc.add_options()
      ("help", "print help message")
      ("type,t", po::value<std::string>(&node_type)->required(), "Type of the perception node. Allowed values: 'gripper' and 'scene'")
    ;

    po::positional_options_description p;
    po::store(po::command_line_parser(argc, argv).
    options(desc).allow_unregistered().run(), vm); // Allow unknown parameters 

    if (vm.count("help")) {
      std::cout << "Usage: suturo_perception_node -t NODE_TYPE" << std::endl;
      std::cout << desc << std::endl << std::endl;
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);
  }
  catch (std::exception &e)
  {
    std::cout << "Usage: suturo_perception_node -t NODE_TYPE" << std::endl;
    std::cout << desc << std::endl;
    std::cerr << "Error: " << e.what() << std::endl << std::endl;
    return false;
  }
  catch(...)
  {
    std::cout << "Usage: suturo_perception_node -t NODE_TYPE" << std::endl;
    std::cout << desc << std::endl << std::endl;
    std::cerr << "Unknown error!" << std::endl;
    return false;
  } 

  if (node_type == "gripper")
  {
    imageTopic = "/suturo/perception/euroc_tcp_image";
    cloudTopic = "/suturo/perception/euroc_tcp_cloud";
    node_type_enum = SuturoPerceptionNode::GRIPPER;
  }
  else
  if (node_type == "scene")
  {
    imageTopic = "/suturo/perception/euroc_scene_image";
    cloudTopic = "/suturo/perception/euroc_scene_cloud";
    node_type_enum = SuturoPerceptionNode::SCENE;
  }
  else
  {
    std::cout << "Usage: suturo_perception_node -t NODE_TYPE" << std::endl;
    std::cout << desc << std::endl << std::endl;
    std::cerr << "Invalid node type specified" << std::endl;
    return false;
  }
    

  // start up node
  ros::init(argc, argv, "suturo_perception_"+node_type+"_node");
  ros::NodeHandle nh;

  ROS_INFO("Image topic is: %s", imageTopic.c_str());
  ROS_INFO("Cloud topic is: %s", cloudTopic.c_str());
  ROS_INFO("Node type is: %s", node_type.c_str());

  SuturoPerceptionNode node(nh, imageTopic, cloudTopic, node_type_enum);

  ROS_INFO("                    _____ ");
  ROS_INFO("                   |     | ");
  ROS_INFO("                   | | | | ");
  ROS_INFO("                   |_____| ");
  ROS_INFO("             ____ ___|_|___ ____ ");
  ROS_INFO("            ()___)         ()___) ");
  ROS_INFO("            // /|           |\\ \\\\ ");
  ROS_INFO("           // / |           | \\ \\\\ ");
  ROS_INFO("          (___) |___________| (___) ");
  ROS_INFO("          (___)   (_______)   (___) ");
  ROS_INFO("          (___)     (___)     (___) ");
  ROS_INFO("          (___)      |_|      (___) ");
  ROS_INFO("          (___)  ___/___\\___   | | ");
  ROS_INFO("           | |  |           |  | | ");
  ROS_INFO("           | |  |___________| /___\\ ");
  ROS_INFO("          /___\\  |||     ||| //   \\\\ ");
  ROS_INFO("         //   \\\\ |||     ||| \\\\   // ");
  ROS_INFO("         \\\\   // |||     |||  \\\\ // ");
  ROS_INFO("          \\\\ // ()__)   (__() ");
  ROS_INFO("                ///       \\\\\\ ");
  ROS_INFO("               ///         \\\\\\ ");
  ROS_INFO("             _///___     ___\\\\\\_ ");
  ROS_INFO("            |_______|   |_______| ");

  ROS_INFO("   ____  __  __ ______  __  __   ___   ____                            ");
  ROS_INFO("  / __/ / / / //_  __/ / / / /  / _ \\ / __ \\                           ");
  ROS_INFO(" _\\ \\  / /_/ /  / /   / /_/ /  / , _// /_/ /                           ");
  ROS_INFO("/___/_ \\____/_ /_/__  \\____/  /_/|_| \\____/______   ____  ____    _  __");
  ROS_INFO("  / _ \\  / __/  / _ \\ / ___/  / __/  / _ \\/_  __/  /  _/ / __ \\  / |/ /");
  ROS_INFO(" / ___/ / _/   / , _// /__   / _/   / ___/ / /    _/ /  / /_/ / /    / ");
  ROS_INFO("/_/    /___/  /_/|_| \\___/  /___/  /_/    /_/    /___/  \\____/ /_/|_/  ");
  ROS_INFO(" ~ suturo_perception_%s_node", node_type.c_str());
                                                                       
  // ROS_INFO("           suturo_perception READY");
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return (0);
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
