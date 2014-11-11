#include <perception_utils/node_status.hpp>

#include <ros/ros.h>
#include <perception_utils/logger.h>
#include <boost/program_options.hpp>

namespace po = boost::program_options;
using namespace boost;
using namespace suturo_perception;

int main(int argc, char **argv)
{
	int task_type;
	// "HashMap" for program parameters
  po::variables_map vm;
  try
  {
    // Declare the supported options.
    po::options_description desc("Allowed options");
    desc.add_options()
      ("help", "produce help message")
      ("task,t", po::value<int>(&task_type)->required(), "The task type. See 'rosmsg_show suturo_msgs/Task' for allowed values (constants TASK_*)")
    ;

    po::positional_options_description p;
    // po::store(po::command_line_parser(argc, argv).
    // options(desc).positional(p).run(), vm); 
    po::store(po::command_line_parser(argc, argv).
    options(desc).allow_unregistered().run(), vm); // Allow unknown parameters 

    if (vm.count("help")) {
      std::cout << "Usage: publish_required_nodes -t TASK_NUM" << std::endl << std::endl;
      std::cout << desc << "\n";
      return 1;
    }

    // Put notify after the help check, so help is display even
    // if required parameters are not given
    po::notify(vm);

  }
  catch(std::exception& e)
  {
    std::cout << "Usage: publish_required_nodes -t TASK_NUM" << std::endl << std::endl;
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  }
  catch(...)
  {
    std::cerr << "Unknown error!" << "\n";
    return false;
  } 

  ros::init(argc, argv, "publish_required_nodes");
	ros::NodeHandle n;
	
	NodeStatus node_status(n);
	node_status.publishRequiredNodes(task_type);
	
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}