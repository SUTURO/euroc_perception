#include "suturo_perception_pipeline/pipeline.h"

#include <suturo_perception_match_cuboid/cuboid_matcher.h>
#include <suturo_perception_centroid_calc/centroid_calc.h>
#include <suturo_perception_shape_detection/shape_detector.h>
#include <suturo_perception_color_analysis/color_analysis.h>
#include <suturo_perception_cad_recognition/model_pose_estimation.h>
#include <suturo_perception_height_calculation/height_calculation.h>

#include <iostream>
#include <fstream>
#include <boost/asio.hpp>

using namespace suturo_perception;

Pipeline::Pipeline()
{
}

Capability* 
Pipeline::instantiateCapability(CapabilityType type, PipelineData::Ptr data, PipelineObject::Ptr obj, bool enabled)
{
  switch (type)
  {
    case CUBOID_MATCHER:
    return new CuboidMatcher(data, obj);
    break;  

    case CENTROID_CALC:
    return new CentroidCalc(data, obj);
    break;

    case SHAPE_DETECTOR:
    return new ShapeDetector(data, obj);
    break;

    case COLOR_ANALYSIS:
    return new ColorAnalysis(data, obj);
    break;

    case MODEL_POSE:
    {
      // TODO: move this stuff into ModelPoseEstimation.execute()
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      // Simple Testcase for now - Use only one model!!!!
      //
      // Prepare the model that should be matched against the input cloud
      boost::shared_ptr<std::vector<suturo_msgs::Object> > objects(new std::vector<suturo_msgs::Object>);
      // return mpe;
      return new ModelPoseEstimation(objects, data, obj);

      break;
    }

    case HEIGHT_CALCULATION:
    return new HeightCalculation(data, obj);
    break;

    // INFO: ADD YOUR NEW CAPABILITY HERE (don't forget to include it)

    default:
    return NULL;
    break;
  }
}

void
Pipeline::execute(PipelineData::Ptr pipeline_data, PipelineObject::VecPtr pipeline_objects)
{
  // Execution pipeline
  // Each capability provides an enrichment for the pipelineObject

  // initialize threadpool
  boost::asio::io_service ioService;
  boost::thread_group threadpool;
  std::auto_ptr<boost::asio::io_service::work> work(
    new boost::asio::io_service::work(ioService));

  // Add worker threads to threadpool
  for(int i = 0; i < pipeline_data->numThreads_; ++i)
  {
    threadpool.create_thread(
      boost::bind(&boost::asio::io_service::run, &ioService)
      );
  }

  // Number of objects to process
  int object_cnt = pipeline_objects.size();
  
  // Available Capabilities
  int avail_capabilities = LAST_CAPABILITY;
  
  // Initialize Capabilities
  std::vector<std::vector<Capability*> > capabilities;
  for (int i = 0; i < object_cnt; i++)
  {
    std::vector<Capability*> object_capabilities;
    for (int j = 0; j < avail_capabilities; j++)
    {
      Capability *cap = instantiateCapability((CapabilityType)j, pipeline_data, pipeline_objects[i]);
      int cap_enabled = capabilityEnabled(pipeline_data->request_parameters_, cap->getName());
      if (cap_enabled > -1)
      {
        //logger.logInfo((boost::format("Capability %s enabled: %s") % cap->getName() % cap_enabled).str());
        cap->setEnabled(cap_enabled);
      }
      object_capabilities.push_back(cap);
    }
    capabilities.push_back(object_capabilities);
  }
  
  // post work to threadpool
  for (int i = 0; i < object_cnt; i++) 
  {
    for (int j = 0; j < avail_capabilities; j++)
    {
      if (capabilities[i][j]->isEnabled())
        ioService.post(boost::bind(&Capability::pipelineExecute, capabilities[i][j]));
    }
  }
  
  //boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  // wait for thread completion.
  // destroy the work object to wait for all queued tasks to finish
  work.reset();
  ioService.run();
  threadpool.join_all();
  
  // Deinitialize Capabilities
	//std::stringstream capabilityExecutionTimes;
	//capabilityExecutionTimes << "~~~" << std::endl;
  for (int i = 0; i < object_cnt; i++) 
  {
		//capabilityExecutionTimes << "Object " << i << std::endl;
    for (int j = 0; j < avail_capabilities; j++)
    {
			//capabilityExecutionTimes << "  Time for " << capabilities[i][j]->getName() << ": " << capabilities[i][j]->getExecutionTime() << std::endl;
      delete capabilities[i][j];
    }
  }
  //std::ofstream capExecTimeFile;
	//capExecTimeFile.open("/tmp/perception_pipeline_execution_times", std::ios::out | std::ios::app);
	//capExecTimeFile << capabilityExecutionTimes.str();
	//capExecTimeFile.close();
}

int
Pipeline::capabilityEnabled(std::string capability_settings, std::string capability_name)
{
  std::vector<std::string> enabled_capabilities = split(capability_settings, ",()");
  if (enabled_capabilities.size() == 0)
    return -1;
  if (capability_settings.compare("get") == 0)
    return -1;
  return std::find(enabled_capabilities.begin(), enabled_capabilities.end(), capability_name) != enabled_capabilities.end()==0?0:1;
}

// taken from: http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
std::vector<std::string> &Pipeline::split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

// taken from: http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
std::vector<std::string> Pipeline::split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}

std::vector<std::string> Pipeline::split(const std::string &s, const std::string &delims)
{
  std::vector<std::string> ret;
  std::string tmp;
  for (int i = 0; i < s.size(); i++)
  {
    bool isDelim = false;
    for (int j = 0; j < delims.size(); j++)
    {
      if (s[i] == delims[j])
        isDelim = true;
    }
    if (isDelim)
    {
      ret.push_back(tmp);
      tmp = "";
    }
    else
    {
      tmp += s[i];
    }
  }
  ret.push_back(tmp);
  return ret;
}
