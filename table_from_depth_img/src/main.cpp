#include "ros/ros.h"
#include <ros/package.h>
#include "table_from_depth_image_node.h"


int main (int argc, char** argv)
{
  ros::init(argc, argv, "table_from_depth_image_node");
  ros::NodeHandle nh;

  std::string cloudTopic = "/suturo/euroc_scene_cloud";
  ROS_INFO("Cloud topic is: %s", cloudTopic.c_str());

  TableFromDepthImageNode node(nh, cloudTopic);

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
                                                                       
  // ROS_INFO("           suturo_perception READY");
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return (0);
}
// vim: tabstop=2 expandtab shiftwidth=2 softtabstop=2: 
